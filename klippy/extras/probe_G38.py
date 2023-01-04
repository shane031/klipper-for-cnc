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
    This class registers a command to home an extruder's stepper.
    This is made possible due to a change in the Extruder class,
    which can now add an endstop to the extruder stepper, if a
    config parameter is provided.

    The "toolhead" passed to the PrinterHoming.manual_home method
    is not entirely "virtual". All methods are defined here, but
    some of them call the methods of the actual toolhead.
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

        # Register commands
        gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("G38.2",
                                    self.cmd_PROBE_G38,
                                    desc=self.cmd_PROBE_G38_help)
        # TODO: register the rest of the G38 probing commands.
    
    cmd_PROBE_G38_help = "Probe towards workpiece with error"
    def cmd_PROBE_G38(self, gcmd):
        pass
        

def load_config(config):
    return ProbeG38(config)
