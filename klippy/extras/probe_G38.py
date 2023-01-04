# Z-Probe support
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# TODO: check if this is useful.
# from . import manual_probe

import logging
import pins
from . import probe


class ProbeEndstopWrapperG38(probe.ProbeEndstopWrapper):
    def __init__(self, config):
        # Instantiate the base "ProbeEndstopWrapper" class, as usual
        super(ProbeEndstopWrapperG38, self).__init__(config)

        self.printer = config.get_printer()

        # NOTE: recovery stuff
        self.recovery_time = config.getfloat('recovery_time', 0.4, minval=0.)

        # NOTE: add XY steppers too
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)

    # Overwrite only the "probe_prepare" method, to include the dwell.
    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self.lower_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'
            
        # NOTE: borrowed code from "smart_effector", trying to
        #       avoid the "Probe triggered prior to movement" error.
        if self.recovery_time:
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.dwell(self.recovery_time)

    # NOTE: trying to solve the "Probe triggered prior to movement" issue.
    def _handle_mcu_identify(self):
        logging.info(f"\n\n" + "ProbeEndstopWrapperG38._handle_mcu_identify activated (XYZ axes)" + "\n\n")
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('x') or stepper.is_active_axis('y') or stepper.is_active_axis('z'):
                self.add_stepper(stepper)

class ProbeG38:
    """
    ! WARNING EXPERIMENTAL

    This class registers G38 commands to probe in general directions.

    From LinuxCNC: https://linuxcnc.org/docs/2.6/html/gcode/gcode.html
        - G38.2 - (True/True) probe toward workpiece, stop on contact, signal error if failure.
        - G38.3 - (True/False) probe toward workpiece, stop on contact.
        - G38.4 - (False/True) probe away from workpiece, stop on loss of contact, signal error if failure.
        - G38.5 - (False/False) probe away from workpiece, stop on loss of contact.
    
    For now, only the G38.2 and G38.3 commands have been implemented.
        
    ! Known problems:

    ! "Communication timeout during homing probe"
    ! G38.2 X150 F10
    TODO: This i have no idea how to fix.

    !   "Probe triggered prior to movement"
    !   G38.3 X10 F10
    From: https://www.klipper3d.org/Config_Reference.html#smart_effector
        A delay between the travel moves and the probing moves in seconds. A fast
        travel move prior to probing may result in a spurious probe triggering.
        This may cause 'Probe triggered prior to movement' errors if no delay
        is set. Value 0 disables the recovery delay.
    Unfortunately the "recovery_time"/dwell thing did not help.
    The error is raised by "check_no_movement", called by "probing_move" at
    the end of the move; which checks if ths start and trigger positions of the
    steppers are the same.
    Perhaps the trigger positions are not updated correctly, because the 
    endstops are associated to only one stepper: the Z.
    Indeed, the probe finishes without errors for a Z probe move.

    The "add_stepper" function in ProbeEndstopWrapper is called by "_handle_mcu_identify",
    which responds to the 'klippy:mcu_identify' event.



    """
    def __init__(self, config):
        # NOTE: because the "config" is passed to PrinterProbe and ProbeEndstopWrapper,
        #       it will require all the parameters that they require, plus the ones specific
        #       to this class.

        # NOTE: instantiate:
        #       -   "ProbeEndstopWrapper": Endstop wrapper that enables probe specific features.
        #       -   "PrinterProbe": ?
        #self.probe = probe.PrinterProbe(config, probe.ProbeEndstopWrapper(config))
        self.probe = probe.PrinterProbe(config, ProbeEndstopWrapperG38(config))
        self.printer = config.get_printer()

        # NOTE: not setup by "load_config", not needed either.
        #self.probe_name = config.get_name().split()[1]

        # NOTE: Override some things from the PrinterProbe init.
        # NOTE: They are no longer needed.
        #self.probe_pos = config.getfloat('endstop_position', self.probe.speed)
        #self.probe.z_position = self.probe_pos

        # NOTE: configure whether te move will be in absolute or relative coordinates
        #self.absolute_coord = config.getboolean('absolute_coord', True)

        # NOTE: Dummy position vector, populated later.
        self.last_position = [None, None, None, None]

        # NOTE: default probing speed
        self.speed = 100

        # NOTE: recovery stuff
        self.recovery_time = config.getfloat('recovery_time', 0.4, minval=0.)

        # NOTE: Register commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("G38.2",
                                    self.cmd_PROBE_G38_2,
                                    when_not_ready=False,
                                    desc=self.cmd_PROBE_G38_2_help)
        self.gcode.register_command("G38.3",
                                    self.cmd_PROBE_G38_3,
                                    when_not_ready=False,
                                    desc=self.cmd_PROBE_G38_3_help)
        # TODO: register the rest of the G38 probing commands.
        
    cmd_PROBE_G38_3_help = "Probe towards workpiece without error."
    def cmd_PROBE_G38_3(self, gcmd):
        self.cmd_PROBE_G38_2(gcmd, error_out=False)

    cmd_PROBE_G38_2_help = "Probe towards workpiece with error."
    def cmd_PROBE_G38_2(self, gcmd, error_out=True):
        # NOTE: Get the toolhead's last position.
        #       This will be updated below.
        toolhead = self.printer.lookup_object('toolhead')
        self.last_position = toolhead.get_position()

        # NOTE: configure whether te move will be in absolute 
        #       or relative coordinates. Respect the G90/G91 setting.
        gcode_move = self.printer.lookup_object('gcode_move')
        self.absolute_coord = gcode_move.absolute_coord

        # NOTE: also get the "base position". This is required to compute
        #       the absolute move, ¿relative to it? Weird...
        self.base_position = gcode_move.base_position

        # NOTE: Dummy objects for the G1 command parser
        self.speed_factor = 1

        # NOTE: probing axes
        probe_axes = []

        # NOTE: coordinate code parser copied from "cmd_G1" at "gcode_move.py".
        params = gcmd.get_command_parameters()
        try:
            for pos, axis in enumerate('XYZ'):
                if axis in params:
                    v = float(params[axis])
                    if not self.absolute_coord:
                        # value relative to position of last move
                        self.last_position[pos] += v
                    else:
                        # value relative to base coordinate position
                        self.last_position[pos] = v + self.base_position[pos]
                    # NOTE: register which axes are being probed
                    probe_axes.append(axis.lower())  # Append "X", "Y", or "Z".
            if 'E' in params:
                v = float(params['E']) * self.extrude_factor
                if not self.absolute_coord or not self.absolute_extrude:
                    # value relative to position of last move
                    self.last_position[3] += v
                else:
                    # value relative to base coordinate position
                    self.last_position[3] = v + self.base_position[3]
                # NOTE: register which axes are being probed
                probe_axes.append("extruder")  # Append "extruder"
            if 'F' in params:
                gcode_speed = float(params['F'])
                if gcode_speed <= 0.:
                    raise gcmd.error("Invalid speed in '%s'"
                                     % (gcmd.get_commandline(),))
                self.speed = gcode_speed * self.speed_factor
        except ValueError as e:
            raise gcmd.error(f"ProbeG38: Unable to parse move {gcmd.get_commandline()} with exception: {str(e)}")
        
        # NOTE: "move_with_transform" is just "toolhead.move":
        # self.move_with_transform(self.last_position, self.speed)

        # TODO: should this go here?
        if self.recovery_time:
            toolhead.dwell(self.recovery_time)
        
        # NOTE: my probe works!
        self.probe_g38(pos=self.last_position, speed=self.speed, error_out=error_out, gcmd=gcmd, probe_axes=probe_axes)

    def probe_g38(self, pos, speed, error_out, gcmd, probe_axes=None):
        # NOTE: code copied from "probe._probe".

        toolhead = self.printer.lookup_object('toolhead')

        # TODO: rethink if homing is neccessary for homing.
        # curtime = self.printer.get_reactor().monotonic()
        # if 'z' not in toolhead.get_status(curtime)['homed_axes']:
        #     raise self.printer.command_error("Must home before probe")
        
        phoming = self.printer.lookup_object('homing')

        # NOTE: This is no longer necessary, because I've passed
        #       the "pos" argument from "cmd_PROBE_G38_2".
        # pos = toolhead.get_position()

        # NOTE: This is also no longer necessary.
        # NOTE: "self.z_position" is equal to the "min_position"
        #       parameter from the "z_stepper" section.
        #       It is used to override the Z component of the 
        #       current toolhead position, probably to generate
        #       the target coordinates for the homing move.
        # pos[2] = self.z_position
        
        try:
            # NOTE: This probe method uses "phoming.probing_move",
            #       passing it "mcu_probe" which is an instance of 
            #       "ProbeEndstopWrapper", a wrapper for the probes'
            #       MCU_endstop object.
            # NOTE: This is in contrast to "phoming.manual_home",
            #       which additionally requires a toolhead object.
            #       It turns out that, if not provided, HomingMove
            #       will get the main toolhead by lookup and use it.
            # NOTE: the method is passed "pos", which is "min_position"
            #       parameter from the "z_stepper" section, and the
            #       current XYE toolhead coordinates (see notes above). 
            # NOTE: I had to add a "check_triggered" argument to 
            #       "probing_move" for G38.3 to work properly.
            logging.info(f"\n\n" + "probe_g38 probing with axes: " + str(probe_axes) + "\n\n")
            epos = phoming.probing_move(mcu_probe=self.probe.mcu_probe,
                                        pos=pos,
                                        speed=speed,
                                        check_triggered=error_out,
                                        probe_axes=probe_axes)

        except self.printer.command_error as e:
            # NOTE: the "fail" logic of the G38 gcode could be
            #       based on this behaviour.
            reason = str(e)
            
            # NOTE: to respect the original logic, only "timeout" errors
            #       can be ignored. Else, the error should be logged with
            #       the "command_error" method, as always.
            if "Timeout during endstop homing" in reason:
                reason += HINT_TIMEOUT
                if error_out:
                    # NOTE: log the error as usual if it was requested.
                    raise self.printer.command_error(reason)
                else:
                    # NOTE: log as a "gcmd response"
                    gcmd.respond_info("G38 timeout without error, reason: " + reason)
            else:
                # NOTE: log the error as usual if it is was not a timeout error.
                raise self.printer.command_error(reason)
        
        self.gcode.respond_info("probe at %.3f,%.3f is z=%.6f"
                                % (epos[0], epos[1], epos[2]))
        
        # TODO: find out why it returns the fourth position.
        return epos[:3]


def load_config(config):
    return ProbeG38(config)
