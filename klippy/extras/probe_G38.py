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

        # NOTE: Register commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("G38.2",
                                    self.cmd_PROBE_G38_2,
                                    when_not_ready=False,
                                    desc=self.cmd_PROBE_G38_2_help)
        # TODO: register the rest of the G38 probing commands.
    
    cmd_PROBE_G38_2_help = "Probe towards workpiece with error."
    def cmd_PROBE_G38_2(self, gcmd):
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
        self.probe_g38(self.last_position, self.speed)

    def probe_g38(self, pos, speed):
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
            epos = phoming.probing_move(self.probe.mcu_probe, pos, speed)

        except self.printer.command_error as e:
            # NOTE: the "fail" logic of the G38 gcode could be
            #       based on this behaviour.
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += HINT_TIMEOUT
            raise self.printer.command_error(reason)
        
        self.gcode.respond_info("probe at %.3f,%.3f is z=%.6f"
                                % (epos[0], epos[1], epos[2]))
        
        # TODO: find out why it returns the fourth position.
        return epos[:3]


def load_config(config):
    return ProbeG38(config)
