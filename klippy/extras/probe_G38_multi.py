# Z-Probe support
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# TODO: check if this is useful.
# from . import manual_probe

import logging
import pins
from . import probe, ProbeEndstopWrapperG38, ProbeG38

class ProbeG38multi(ProbeG38):
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

        # NOTE: save original probing config logic.
        #       This logic is used at "_home_cmd.send()" in "mcu.py"
        #       to make the low-level "endstop_home" MCU command.
        # self.invert_config = self.probe._invert

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
        
        # NOTE: get name of the probe from the config
        self.probe_name = config.get_name().split()[1]
        
        # NOTE: From LinuxCNC: https://linuxcnc.org/docs/2.6/html/gcode/gcode.html
        #       - G38.2 - Probe toward workpiece, stop on contact, signal error if failure.
        self.gcode.register_mux_command("G38.2", "PROBE",
                                        self.probe_name, self.cmd_PROBE_G38_2,
                                        when_not_ready=False,
                                        desc=self.cmd_PROBE_G38_2_help)
        #       - G38.3 - Probe toward workpiece, stop on contact.
        self.gcode.register_mux_command("G38.3", "PROBE",
                                        self.probe_name, self.cmd_PROBE_G38_3,
                                        when_not_ready=False,
                                        desc=self.cmd_PROBE_G38_3_help)
        #       - G38.4 - Probe away from workpiece, stop on loss of contact, signal error if failure.
        self.gcode.register_mux_command("G38.4", "PROBE",
                                        self.probe_name, self.cmd_PROBE_G38_4,
                                        when_not_ready=False,
                                        desc=self.cmd_PROBE_G38_4_help)
        #       - G38.5 - Probe away from workpiece, stop on loss of contact.
        self.gcode.register_mux_command("G38.5", "PROBE",
                                        self.probe_name, self.cmd_PROBE_G38_5,
                                        when_not_ready=False,
                                        desc=self.cmd_PROBE_G38_5_help)
    
def load_config_prefix(config):
    return ProbeG38multi(config)
