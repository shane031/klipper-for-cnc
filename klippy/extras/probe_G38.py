# Z-Probe support
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import pins
from . import probe

# TODO: check if this is useful.
# from . import manual_probe

class ProbeG38:
    """
    ! WARNING EXPERIMENTAL
    This class registers G38 commands to probe in general directions.
    """
    def __init__(self, config):
        # NOTE: because the "config" is passed to PrinterProbe and ProbeEndstopWrapper,
        #       it will require all the parameters that they require, plus the ones specific
        #       to this class.

        # NOTE: instantiate:
        #       -   "ProbeEndstopWrapper": Endstop wrapper that enables probe specific features.
        #       -   "PrinterProbe": ?
        self.probe = probe.PrinterProbe(config, probe.ProbeEndstopWrapper(config))
        self.printer = config.get_printer()
        self.probe_name = config.get_name().split()[1]

        # NOTE: override some things from the PrinterProbe init.
        self.probe_pos = config.getfloat('endstop_position', self.probe.speed)
        self.probe.z_position = self.probe_pos

        # NOTE: configure whether te move will be in absolute or relative coordinates
        #self.absolute_coord = config.getboolean('absolute_coord', True)

        # NOTE: Dummy position vector, populated later.
        self.last_position = [None, None, None, None]

        # NOTE: Register commands
        gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("G38.2",
                                    self.cmd_PROBE_G38_2,
                                    when_not_ready=False,
                                    desc=self.cmd_PROBE_G38_2_help)
        # TODO: register the rest of the G38 probing commands.
    
    cmd_PROBE_G38_2_help = "Probe towards workpiece with error."
    def cmd_PROBE_G38_2(self, gcmd):
        # NOTE: get the toolhead's last position.
        toolhead = self.printer.lookup_object('toolhead')
        self.last_position = toolhead.get_position()

        # NOTE: configure whether te move will be in absolute 
        #       or relative coordinates. Respect the G90/G91 setting.
        gcode_move = self.printer.lookup_object('gcode_move')
        self.absolute_coord = gcode_move.absolute_coord

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
            if 'E' in params:
                v = float(params['E']) * self.extrude_factor
                if not self.absolute_coord or not self.absolute_extrude:
                    # value relative to position of last move
                    self.last_position[3] += v
                else:
                    # value relative to base coordinate position
                    self.last_position[3] = v + self.base_position[3]
            if 'F' in params:
                gcode_speed = float(params['F'])
                if gcode_speed <= 0.:
                    raise gcmd.error("Invalid speed in '%s'"
                                     % (gcmd.get_commandline(),))
                self.speed = gcode_speed * self.speed_factor
        except ValueError as e:
            raise gcmd.error("Unable to parse move '%s'"
                             % (gcmd.get_commandline(),))
        
        # NOTE: "move_with_transform" is just "toolhead.move":
        # self.move_with_transform(self.last_position, self.speed)
        # TODO: define the "_probe move here".

        
        

def load_config(config):
    return ProbeG38(config)
