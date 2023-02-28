# This file may be distributed under the terms of the GNU GPLv3 license.

import math, logging, importlib
import chelper

# WARNING EXPERIMENTAL!

class ExtraToolHead():
    """Extra axis class.

    Example config:
    
    [extra_toolhead abc]
    kinematics: cartesian_abc
    axis: ABC
    max_velocity: 5000
    max_z_velocity: 250
    max_accel: 1000
    """
    def __init__(self, config):
        # NOTE: React to G1 move commands.
        self.printer.register_event_handler("gcode_move:parsing_move_command",
                                            self.handle_G1)
        
        # NOTE: amount of non-extruder axes: XYZ=3, XYZABC=6.
        self.axis_count = 3
        self.axis_names = 'ABC'
        
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.step_generators = []
        
        gcode = self.printer.lookup_object('gcode')
        self.Coord = gcode.Coord
        
        # Create kinematics class
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
        
    def handle_G1(self, gcmd, params):
        pass

def load_config_prefix(config):
    """Replaces toolhead.add_printer_objects
    def add_printer_objects(config):
        config.get_printer().add_object('toolhead', ToolHead(config))
        kinematics.extruder.add_printer_objects(config)
    """
    return ExtraToolHead(config)

