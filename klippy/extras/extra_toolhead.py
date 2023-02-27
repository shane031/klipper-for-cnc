# Code for coordinating events on the printer toolhead
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, importlib
import mcu, chelper, kinematics.extruder
import time

# NOTE: import the original ToolHead class.
from toolhead import ToolHead

# Common suffixes: _d is distance (in mm), _v is velocity (in
#   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
#   seconds), _r is ratio (scalar between 0.0 and 1.0)

# Class to track each move request: Move
from toolhead import Move

LOOKAHEAD_FLUSH_TIME = 0.250

# Class to track a list of pending move requests and to facilitate
# "look-ahead" across moves to reduce acceleration between moves.
from toolhead import MoveQueue

# NOTE: DripModeEndSignal(Exception)
from toolhead import DripModeEndSignal

# TODO: this quantity is undocumented.
MIN_KIN_TIME = 0.100

# NOTE: Some insight on this parameter may be available here:
#       https://github.com/Klipper3d/klipper/commit/7ca86f17232e5e0653de512b6322c301b153919c
MOVE_BATCH_TIME = 0.500

SDS_CHECK_TIME = 0.001 # step+dir+step filter in stepcompress.c

DRIP_SEGMENT_TIME = 0.050
DRIP_TIME = 0.100

# Main code to track events (and their timing) on the printer toolhead
class ExtraToolHead(ToolHead):
    """Extra toolhead class.

    Example config:
    
    [extra_toolhead abc]
    kinematics: cartesian_abc
    max_velocity: 5000
    max_z_velocity: 250
    max_accel: 1000
    """
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.all_mcus = [
            m for n, m in self.printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]
        self.can_pause = True
        if self.mcu.is_fileoutput():
            self.can_pause = False
        self.move_queue = MoveQueue(self)
        self.commanded_pos = [0., 0., 0., 0.]
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
        
        # Velocity and acceleration control
        self.max_velocity = config.getfloat('max_velocity', above=0.)
        self.max_accel = config.getfloat('max_accel', above=0.)
        self.requested_accel_to_decel = config.getfloat(
            'max_accel_to_decel', self.max_accel * 0.5, above=0.)
        self.max_accel_to_decel = self.requested_accel_to_decel
        self.square_corner_velocity = config.getfloat(
            'square_corner_velocity', 5., minval=0.)
        self.junction_deviation = 0.
        self._calc_junction_deviation()
        
        # Print time tracking
        self.buffer_time_low = config.getfloat(
            'buffer_time_low', 1.000, above=0.)
        self.buffer_time_high = config.getfloat(
            'buffer_time_high', 2.000, above=self.buffer_time_low)
        self.buffer_time_start = config.getfloat(
            'buffer_time_start', 0.250, above=0.)
        self.move_flush_time = config.getfloat(
            'move_flush_time', 0.050, above=0.)
        self.print_time = 0.
        self.special_queuing_state = "Flushed"
        self.need_check_stall = -1.
        self.flush_timer = self.reactor.register_timer(self._flush_handler)
        self.move_queue.set_flush_time(self.buffer_time_high)
        self.idle_flush_print_time = 0.
        self.print_stall = 0
        self.drip_completion = None
        
        # Kinematic step generation scan window time tracking
        self.kin_flush_delay = SDS_CHECK_TIME
        self.kin_flush_times = []
        self.force_flush_time = self.last_kin_move_time = 0.
        
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.step_generators = []
        
        # Create kinematics class
        gcode = self.printer.lookup_object('gcode')
        self.Coord = gcode.Coord
        self.extruder = kinematics.extruder.DummyExtruder(self.printer)
        # NOTE: get "kinematics" name from "[printer]".
        kin_name = config.get('kinematics')
        try:
            mod = importlib.import_module('kinematics.' + kin_name)
            self.kin = mod.load_kinematics(self, config)
        except config.error as e:
            raise
        except self.printer.lookup_object('pins').error as e:
            raise
        except:
            msg = "Error loading kinematics '%s'" % (kin_name,)
            logging.exception(msg)
            raise config.error(msg)
        
        # Register commands
        gcode.register_command('G4', self.cmd_G4)
        gcode.register_command('M400', self.cmd_M400)
        gcode.register_command('SET_VELOCITY_LIMIT',
                               self.cmd_SET_VELOCITY_LIMIT,
                               desc=self.cmd_SET_VELOCITY_LIMIT_help)
        gcode.register_command('M204', self.cmd_M204)
        
        # Load some default modules
        modules = ["gcode_move", "homing", "idle_timeout", "statistics",
                   "manual_probe", "tuning_tower"]
        for module_name in modules:
            self.printer.load_object(config, module_name)


def load_config_prefix(config):
    """Replaces toolhead.add_printer_objects
    def add_printer_objects(config):
        config.get_printer().add_object('toolhead', ToolHead(config))
        kinematics.extruder.add_printer_objects(config)
    """
    return ExtraToolHead(config)

