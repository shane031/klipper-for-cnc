# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
from kinematics.cartesian import CartKinematics

class CartKinematicsABC(CartKinematics):
    """Kinematics for the ABC axes in the main toolhead class.

    Example config:
    
    [printer]
    kinematics: cartesian
    axis: XYZ  # Optional: XYZ or XYZABC
    kinematics_abc: cartesian_abc # Optional
    max_velocity: 5000
    max_z_velocity: 250
    max_accel: 1000
    
    TODO:
      - The "checks" still have the XYZ logic.
      - Homing is not implemented for ABC.
    """
    def __init__(self, toolhead, config, trapq=None):
        self.printer = config.get_printer()
        
        # Axis names
        self.axis = [3, 4, 5]
        self.axis_names = "".join([toolhead.axis_names[i] for i in self.axis])  # Will get "ABC" from "XYZABC"
        self.axis_count = toolhead.axis_count  # len(self.axis_names)
        logging.info(f"\n\nCartKinematicsABC: starting setup with axes: {self.axis_names}.\n\n")
        
        # Get the trapq
        if trapq is None:
            self.trapq = toolhead.get_abc_trapq()
        else:
            self.trapq = trapq
        
        # Setup axis rails
        # self.dual_carriage_axis = None
        # self.dual_carriage_rails = []
        
        # NOTE: a "PrinterRail" is setup by LookupMultiRail, per each 
        #       of the three axis, including their corresponding endstops.
        # NOTE: The "self.rails" list contains "PrinterRail" objects, which
        #       can have one or more stepper (PrinterStepper/MCU_stepper) objects.
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in self.axis_names.lower()]
        
        # NOTE: this must be "xyz" and not "abc", see "cartesian_stepper_alloc" in C code.
        # for rail, axis in zip(self.rails, self.axis_names.lower()):
        for rail, axis in zip(self.rails, "xyz"):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
        
        for s in self.get_steppers():
            s.set_trapq(self.trapq)
            # TODO: check if this "generator" should be appended to 
            #       the "self.step_generators" list in the toolhead,
            #       or to the list in the new TrapQ.
            #       Using the toolhead for now.
            #       This object is used by "toolhead._update_move_time".
            toolhead.register_step_generator(s.generate_steps)
        
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        # Setup boundary checks
        # NOTE: Returns max_velocity and max_accel from the toolhead's config.
        #       Used below as default values.
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3
        ranges = [r.get_range() for r in self.rails]
        
        # TODO: check if this works with ABC axes, it will result in 
        #       Coord(x=0.0, y=0.0, z=0.0, e=0.0, a=None, b=None, c=None)
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
        
        # Check for dual carriage support
        # if config.has_section('dual_carriage'):
        #     dc_config = config.getsection('dual_carriage')
        #     dc_axis = dc_config.getchoice('axis', {'x': 'x', 'y': 'y'})
        #     self.dual_carriage_axis = {'x': 0, 'y': 1}[dc_axis]
        #     dc_rail = stepper.LookupMultiRail(dc_config)
        #     dc_rail.setup_itersolve('cartesian_stepper_alloc', dc_axis.encode())
        #     for s in dc_rail.get_steppers():
        #         toolhead.register_step_generator(s.generate_steps)
        #     self.dual_carriage_rails = [
        #         self.rails[self.dual_carriage_axis], dc_rail]
        #     self.printer.lookup_object('gcode').register_command(
        #         'SET_DUAL_CARRIAGE', self.cmd_SET_DUAL_CARRIAGE,
        #         desc=self.cmd_SET_DUAL_CARRIAGE_help)
    
    def get_steppers(self):
        # NOTE: The "self.rails" list contains "PrinterRail" objects, which
        #       can have one or more stepper (PrinterStepper/MCU_stepper) objects.
        rails = self.rails
        
        # if self.dual_carriage_axis is not None:
        #     dca = self.dual_carriage_axis
        #     rails = rails[:dca] + self.dual_carriage_rails + rails[dca+1:]
        
        # NOTE: run "get_steppers" on each "PrinterRail" object from 
        #       the "self.rails" list. That method returns the list of
        #       all "PrinterStepper"/"MCU_stepper" objects in the kinematic.
        return [s for rail in rails for s in rail.get_steppers()]
    
    def calc_position(self, stepper_positions):
        return [stepper_positions[rail.get_name()] for rail in self.rails]
    
    def set_position(self, newpos, homing_axes):
        logging.info("\n\n" +
                     f"cartesian_abc.set_position: setting kinematic position of {len(self.rails)} rails " +
                     f"with newpos={newpos} and homing_axes={homing_axes}.\n\n")
        for i, rail in enumerate(self.rails):
            # NOTE: calls "itersolve_set_position".
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)
    
    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None for i in range(self.axis_count + 1)]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        logging.info(f"\n\ncartesian_abc._home_axis: homing  axis={axis} with forcepos={forcepos} and homepos={homepos}\n\n")
        homing_state.home_rails([rail], forcepos, homepos)
    
    def home(self, homing_state):
        # Each axis is homed independently and in order
        toolhead = self.printer.lookup_object('toolhead')
        for axis in homing_state.get_axes():
            # NOTE: support for dual carriage removed.
            self._home_axis(homing_state, axis, self.rails[toolhead.axes_to_xyz(axis)])
    
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
    
    def _check_endstops(self, move):
        logging.info("\n\n" + f"cartesian_abc._check_endstops: triggered on {self.axis_names}/{self.axis} move .\n\n")
        end_pos = move.end_pos
        for i, axis in enumerate(self.axis):
            if (move.axes_d[axis]
                and (end_pos[axis] < self.limits[i][0]
                     or end_pos[axis] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error(f"Must home axis {self.axis_names[i]} first")
                raise move.move_error()
    
    # TODO: Use the original toolhead's z-axis limit here.
    # TODO: Think how to "sync" speeds with the original toolhead,
    #       so far the ABC axis should just mirror the XY.
    def check_move(self, move):
        """Checks a move for validity.
        
        Also limits the move's max speed to the limit of the Z axis if used.

        Args:
            move (tolhead.Move): Instance of the Move class.
        """
        
        limits = self.limits
        apos, bpos = [move.end_pos[axis] for axis in self.axis[:2]]  # move.end_pos[3:6]
        logging.info("\n\n" + f"cartesian_abc.check_move: checking move ending on apos={apos} and bpos={bpos}.\n\n")
        if (apos < limits[0][0] or apos > limits[0][1]
            or bpos < limits[1][0] or bpos > limits[1][1]):
            self._check_endstops(move)
        
        # NOTE: check if the move involves the Z axis, to limit the speed.
        if not move.axes_d[self.axis[3]]:
            # Normal XY move - use defaults
            return
        else:
            self._check_endstops(move)
            # NOTE: removed the "Z" logic here, as it is implemented in 
            #       the XYZ cartesian kinematic check already.
            # Move with Z - update velocity and accel for slower Z axis
            # z_ratio = move.move_d / abs(move.axes_d[2])
            # move.limit_speed(
            #     self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
        return
    
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip(self.axis_names, self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }
    
    # Dual carriage support
    # def _activate_carriage(self, carriage):
    #     toolhead = self.printer.lookup_object('toolhead')
    #     toolhead.flush_step_generation()
    #     dc_rail = self.dual_carriage_rails[carriage]
    #     dc_axis = self.dual_carriage_axis
    #     self.rails[dc_axis].set_trapq(None)
    #     dc_rail.set_trapq(toolhead.get_trapq())
    #     self.rails[dc_axis] = dc_rail
    #     pos = toolhead.get_position()
    #     pos[dc_axis] = dc_rail.get_commanded_position()
    #     toolhead.set_position(pos)
    #     if self.limits[dc_axis][0] <= self.limits[dc_axis][1]:
    #         self.limits[dc_axis] = dc_rail.get_range()
    # cmd_SET_DUAL_CARRIAGE_help = "Set which carriage is active"
    # def cmd_SET_DUAL_CARRIAGE(self, gcmd):
    #     carriage = gcmd.get_int('CARRIAGE', minval=0, maxval=1)
    #     self._activate_carriage(carriage)

def load_kinematics(toolhead, config, trapq=None):
    return CartKinematicsABC(toolhead, config, trapq)
