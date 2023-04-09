# Support for a manual controlled stepper
#
# Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import stepper, chelper
from . import force_move, manual_stepper
import logging

class ManualSpinner(manual_stepper.ManualStepper):
    def __init__(self, config):
        self.printer = config.get_printer()
        
        # NOTE: save the "reactor" object, I need it for timers/spinning.
        self.reactor = self.printer.get_reactor()
        # NOTE: Need this to register the spin move callback appropriately.
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.spin_timer = None
        # TODO: make the spin command delay configurable.
        self.spin_move_delay = 1.0
        self.spin_speed = 0.0
        self.spinning = 0.0

        if config.get('endstop_pin', None) is not None:
            self.can_home = True
            # NOTE: Instantiate a new PrinterRail class from the
            #       stepper.py module.
            self.rail = stepper.PrinterRail(
                config, need_position_minmax=False, default_position_endstop=0.)
            self.steppers = self.rail.get_steppers()
        else:
            self.can_home = False
            self.rail = stepper.PrinterStepper(config)
            self.steppers = [self.rail]
        self.velocity = config.getfloat('velocity', 5., above=0.)
        self.accel = self.homing_accel = config.getfloat('accel', 0., minval=0.)
        self.next_cmd_time = 0.
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.rail.setup_itersolve('cartesian_stepper_alloc', b'x')
        self.rail.set_trapq(self.trapq)
        # Register commands
        stepper_name = config.get_name().split()[1]
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('MANUAL_STEPPER', "STEPPER",
                                   stepper_name, self.cmd_MANUAL_STEPPER,
                                   desc=self.cmd_MANUAL_STEPPER_help)
        gcode.register_mux_command('SPIN_MANUAL_STEPPER', "STEPPER",
                                   stepper_name, self.cmd_SPIN_MANUAL_STEPPER,
                                   desc=self.cmd_SPIN_MANUAL_STEPPER_help)

    # Spin GCODE command
    cmd_SPIN_MANUAL_STEPPER_help = "Spin a manually configured stepper continuously"
    def cmd_SPIN_MANUAL_STEPPER(self, gcmd):
        """Rotate continuously"""
        # Save parameters
        movedist = gcmd.get_float('MOVE', 360.0)
        speed = gcmd.get_float('SPEED', self.velocity)
        accel = gcmd.get_float('ACCEL', self.accel, minval=0.)
        sync = gcmd.get_int('SYNC', 1)
        self.spin_params = (movedist, abs(speed), accel, sync)

        # Run the timer "do_spin_move" callback now (if it is was not running already).
        if not self.spin_speed:
            # To have it turn, first update the speed.
            self.spin_speed = speed
            # Then update the timer, which will then call the "do_spin_move" callback.
            self.reactor.update_timer(self.spin_timer, self.reactor.NOW)
        else:
            # Else it will be done automatically, just update the speed.
            self.spin_speed = speed

    # Continuous rotation move repeat timer.
    def handle_ready(self):
        """Register timer callback for continuous stepper rotation.
        Logic borrowed from "delayed_gcode.py".
        """
        logging.info(f"\n\nmanual_stepper.handle_ready: registering self.spin_timer.\n\n")
        self.spin_timer = self.reactor.register_timer(
            # Callback function.
            self.do_spin_move,
            # Initially the timer should be inactive.
            self.reactor.NEVER)

    def do_start(self, speed, accel, sync=True):
        self.sync_print_time()
        cp = self.rail.get_commanded_position()

        # Acceleration parameters
        accel_t, cruise_t, decel_t = speed / accel, 0.0, 0.0
        start_v, cruise_v, accel = 0.0, 0.0, accel
        
        # Calculate total move time
        movetime = accel_t + cruise_t + decel_t

        # Append move to the trapq
        self.trapq_append_move(
            print_time=self.next_cmd_time,
            accel_t=accel_t, cruise_t=cruise_t, decel_t=decel_t,
            start_pos_x=cp, start_v=start_v, cruise_v=cruise_v, accel=accel)

        # Increment "self.next_cmd_time", call "generate_steps" and "trapq_finalize_moves".
        self.do_final_updates(movetime, sync)

        logging.info(f"\n\ndo_spin_move: sent do_start at next_cmd_time={self.next_cmd_time} and movetime={movetime}")
        return movetime

    def do_cruise(self, dist, speed, sync=True):
        # self.sync_print_time()
        cp = self.rail.get_commanded_position()
        
        # Cruising parameters
        accel_t, cruise_t, decel_t = 0.0, dist / speed, 0.0
        start_v, cruise_v, accel = speed, speed, 0.0

        # Calculate total move time
        movetime = accel_t + cruise_t + decel_t

        # Append move to the trapq
        self.trapq_append_move(
            print_time=self.next_cmd_time,
            accel_t=accel_t, cruise_t=cruise_t, decel_t=decel_t,
            start_pos_x=cp, start_v=start_v, cruise_v=cruise_v, accel=accel)

        # Increment "self.next_cmd_time", call "generate_steps" and "trapq_finalize_moves".
        self.do_final_updates(movetime, sync)

        logging.info(f"\n\ndo_spin_move: sent do_cruise at next_cmd_time={self.next_cmd_time} and movetime={movetime}")
        return movetime

    def do_break(self, speed, decel, sync=True):
        self.sync_print_time()
        cp = self.rail.get_commanded_position()

        # Breaking parameters
        accel_t, cruise_t, decel_t = 0.0, 0.0, speed / decel
        start_v, cruise_v, accel = speed, speed, decel
        # NOTE: Both "start_v" and "cruise_v" must have the value of
        #       the initial speed. If "cruise_v" is set to 0.0, then the
        #       move actually starts from speed 0, and accelerates.

        # Calculate total move time
        movetime = accel_t + cruise_t + decel_t

        # Append move to the trapq
        self.trapq_append_move(
            print_time=self.next_cmd_time,
            accel_t=accel_t, cruise_t=cruise_t, decel_t=decel_t,
            start_pos_x=cp, start_v=start_v, cruise_v=cruise_v, accel=accel)

        # Increment "self.next_cmd_time", call "generate_steps" and "trapq_finalize_moves".
        self.do_final_updates(movetime, sync)

        logging.info(f"\n\ndo_spin_move: sent do_break at next_cmd_time={self.next_cmd_time} and movetime={movetime}")
        return movetime
    
    # Continuous rotation (move repeat) timer callback function.
    def do_spin_move(self, eventtime):
        logging.info(f"\n\ndo_spin_move: called at eventtime={eventtime} with next_cmd_time={self.next_cmd_time}\n\n")
        toolhead = self.printer.lookup_object('toolhead')

        # The default is to never run the timer.
        waketime = self.reactor.NEVER

        # Get the print_time associated with the timer's "present" time.
        est_print_eventtime = toolhead.mcu.estimated_print_time(eventtime)

        # Used for running the timer function a bit before the
        # time for the next stepper move.
        # NEXT_CMD_ANTICIP_TIME = 0.01

        # NOTE: On the different times here:
        #       -   self.next_cmd_time is in "print time" seconds (i.e. MCU time).
        #       -   self.reactor.monotonic is in "system time" seconds (i.e. the Pi's time).
        
        # NOTE: The "self.spin_speed" variable is only updated by the
        # "cmd_SPIN_MANUAL_STEPPER" command.
        
        # Start move (accelerate).
        if self.spin_speed and not self.spinning:
            # If the speed is not 0 or None, queue a rotation command.
            
            # Calculate move coordinates from the last commanded position
            movedist = self.spin_params[0]
            startup_dist = self.break_dist(self.spin_params[1], self.spin_params[2])  # SPEED, ACCEL
            cruise_dist = movedist-startup_dist
            
            # Do start/cruise moves.
            starttime = self.do_start(speed=self.spin_params[1],  # SPEED
                                      accel=self.spin_params[2],  # ACCEL
                                      sync=self.spin_params[3])   # SYNC
            
            # TODO: handle negative displacement with copysign.
            crustime = self.do_cruise(dist=cruise_dist, 
                                      speed=self.spin_params[1],  # SPEED
                                      sync=self.spin_params[3])   # SYNC
            
            # Total move time.
            movetime = starttime + crustime

            # Calculate the time the current move takes.
            time_to_next_move = self.next_cmd_time - est_print_eventtime
            
            # Add a fraction of the move duration to the current system time (actually
            # the time at which this callback was triggered).
            # In this way the next move will be queued during the current move
            # (i.e. before it finishes) to avoid sudden stops between them.
            # waketime = eventtime + time_to_next_move*0.2
            
            # Add a fraction of the move duration to the current system time (actually
            # the time at which this callback was triggered).
            # In this way the next move will be queued at the midpoint of the cruise
            # move that was just queued.
            waketime = eventtime + time_to_next_move - crustime/2.0
            
            # waketime = eventtime + movetime  # This works but cannot be stopped immediately.
            # waketime = self.reactor.monotonic() + self.spin_move_delay
            # waketime = self.reactor.monotonic() + self.next_cmd_time
            
            # Track current status: mark the acceleration sequence as done. 
            self.spinning = self.spin_speed
        
        # Cruise move (uniform motion).
        elif self.spin_speed and self.spinning:
            # If the requested speed is not 0, and the stepper is "spinning", cruise.

            # Do the break
            movetime = self.do_cruise(dist=self.spin_params[0], 
                                      speed=self.spin_params[1],
                                      sync=self.spin_params[3])
            
            
            # Calculate the time the current move takes.
            time_to_next_move = self.next_cmd_time - est_print_eventtime
            
            # Add the move duration to the current system time.
            # In this way the next move will be issued during the current move
            # but also before it finishes.
            # waketime = eventtime + time_to_next_move*0.8

            # Add a fraction of the move duration to the current system time (actually
            # the time at which this callback was triggered).
            # In this way the next move will be queued at the midpoint of the cruise
            # move that was just queued.
            waketime = eventtime + time_to_next_move - movetime*0.75
        
        # Break move (decelerate).
        elif not self.spin_speed and self.spinning:
            # If the requested speed is zero, but the state is still "spinning", signal a breaking move.
            # Calculate breaking distance.
            movedist = self.spin_params[0]
            initial_speed = self.spinning
            break_accel = self.spin_params[2]
            break_dist = self.break_dist(initial_speed, break_accel)
            
            # Do the break
            # TODO: handle negative displacement with copysign.
            crustime = self.do_cruise(dist=movedist - break_dist, 
                                      speed=initial_speed,
                                      sync=self.spin_params[3])
            movetime = self.do_break(speed=initial_speed,
                                     decel=break_accel,
                                     sync=self.spin_params[3])
            
            # Calculate the time the current move takes.
            time_to_next_move = self.next_cmd_time - est_print_eventtime
            
            # Add a large fraction of it to the current system time.
            # In this way the next move will be issued during the current move
            # but also before it finishes.
            # waketime = eventtime + time_to_next_move*0.8
            
            waketime = eventtime + time_to_next_move*1.0

            # Signal break complete
            self.spinning = 0.0

        else:
            logging.info(f"\n\ndo_spin_move: NO CONDITION MATCHED\n\n")    
        
        # Update the timer's next firing time.
        logging.info(f"\n\ndo_spin_move: function ended with waketime={waketime}\n\n")
        return waketime

    def break_dist(self, speed, accel):
        """Calculate acceleration/breaking distance from speed and acceleration."""
        # See: https://en.wikipedia.org/wiki/Acceleration#Uniform_acceleration
        break_distance = (speed**2) / (2*accel)
        # break_time = speed / accel
        return break_distance

def load_config_prefix(config):
    return ManualSpinner(config)
