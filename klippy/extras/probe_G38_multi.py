# Z-Probe support
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# TODO: check if this is useful.
# from . import manual_probe

import logging
import pins
from . import probe, probe_G38

class ProbeG38multi(probe_G38.ProbeG38):
    def __init__(self, config):
        # NOTE: because the "config" is passed to PrinterProbe and ProbeEndstopWrapper,
        #       it will require all the parameters that they require, plus the ones specific
        #       to this class.

        # NOTE: get name of the probe from the config
        self.probe_name = config.get_name().split()[1]

        # NOTE: instantiate:
        #       -   "ProbeEndstopWrapper": Endstop wrapper that enables probe specific features.
        #       -   "PrinterProbe": ?
        #self.probe = probe.PrinterProbe(config, probe.ProbeEndstopWrapper(config))
        self.probe = PrinterProbeMux(config=config,
                                     mcu_probe=probe_G38.ProbeEndstopWrapperG38(config),
                                     mcu_probe_name='probe_'+self.probe_name)
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
        
        # NOTE: From LinuxCNC: https://linuxcnc.org/docs/2.6/html/gcode/gcode.html
        #       - G38.2 - Probe toward workpiece, stop on contact, signal error if failure.
        self.gcode.register_mux_command("LALA", "PNAME",
                                        self.probe_name, self.cmd_PROBE_G38_2,
                                        #when_not_ready=False,
                                        desc=self.cmd_PROBE_G38_2_help)
        #       - G38.3 - Probe toward workpiece, stop on contact.
        self.gcode.register_mux_command("G38.3", "PNAME",
                                        self.probe_name, self.cmd_PROBE_G38_3,
                                        #when_not_ready=False,
                                        desc=self.cmd_PROBE_G38_3_help)
        #       - G38.4 - Probe away from workpiece, stop on loss of contact, signal error if failure.
        self.gcode.register_mux_command("G38.4", "PNAME",
                                        self.probe_name, self.cmd_PROBE_G38_4,
                                        #when_not_ready=False,
                                        desc=self.cmd_PROBE_G38_4_help)
        #       - G38.5 - Probe away from workpiece, stop on loss of contact.
        self.gcode.register_mux_command("G38.5", "PNAME",
                                        self.probe_name, self.cmd_PROBE_G38_5,
                                        #when_not_ready=False,
                                        desc=self.cmd_PROBE_G38_5_help)


class PrinterProbeMux(probe.PrinterProbe):
    def __init__(self, config, mcu_probe, mcu_probe_name='probe'):
        """
        config: ?
        mcu_probe: this is of "ProbeEndstopWrapper" class, which is a wrapper for "MCU_endstop".
        """
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.mcu_probe_name=mcu_probe_name
        self.mcu_probe = mcu_probe
        self.speed = config.getfloat('speed', 5.0, above=0.)
        self.lift_speed = config.getfloat('lift_speed', self.speed, above=0.)
        self.x_offset = config.getfloat('x_offset', 0.)
        self.y_offset = config.getfloat('y_offset', 0.)
        self.z_offset = config.getfloat('z_offset')
        self.probe_calibrate_z = 0.
        self.multi_probe_pending = False
        self.last_state = False
        self.last_z_result = 0.
        self.gcode_move = self.printer.load_object(config, "gcode_move")
        # Infer Z position to move to during a probe
        if config.has_section('stepper_z'):
            zconfig = config.getsection('stepper_z')
            self.z_position = zconfig.getfloat('position_min', 0.,
                                               note_valid=False)
        else:
            pconfig = config.getsection('printer')
            self.z_position = pconfig.getfloat('minimum_z_position', 0.,
                                               note_valid=False)
        # Multi-sample support (for improved accuracy)
        self.sample_count = config.getint('samples', 1, minval=1)
        self.sample_retract_dist = config.getfloat('sample_retract_dist', 2.,
                                                   above=0.)
        atypes = {'median': 'median', 'average': 'average'}
        self.samples_result = config.getchoice('samples_result', atypes,
                                               'average')
        self.samples_tolerance = config.getfloat('samples_tolerance', 0.100,
                                                 minval=0.)
        self.samples_retries = config.getint('samples_tolerance_retries', 0,
                                             minval=0)
        
        # Register z_virtual_endstop pin
        # TODO: study this to implement probing on any direction.
        self.printer.lookup_object('pins').register_chip(self.mcu_probe_name, self)
        
        # Register homing event handlers
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self._handle_homing_move_begin)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self._handle_homing_move_end)
        self.printer.register_event_handler("homing:home_rails_begin",
                                            self._handle_home_rails_begin)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self._handle_home_rails_end)
        self.printer.register_event_handler("gcode:command_error",
                                            self._handle_command_error)
        # Register PROBE/QUERY_PROBE commands
        self.gcode = self.printer.lookup_object('gcode')
        
        self.gcode.register_mux_command('PROBE', 'PROBE_NAME',
                                        self.mcu_probe_name,
                                        self.cmd_PROBE,
                                        desc=self.cmd_PROBE_help)
        
        self.gcode.register_mux_command('QUERY_PROBE', 'PROBE_NAME',
                                        self.mcu_probe_name,
                                        self.cmd_QUERY_PROBE,
                                        desc=self.cmd_QUERY_PROBE_help)
        
        self.gcode.register_mux_command('PROBE_CALIBRATE', 'PROBE_NAME',
                                        self.mcu_probe_name,
                                        self.cmd_PROBE_CALIBRATE,
                                        desc=self.cmd_PROBE_CALIBRATE_help)
        
        self.gcode.register_mux_command('PROBE_ACCURACY', 'PROBE_NAME',
                                        self.mcu_probe_name,
                                        self.cmd_PROBE_ACCURACY,
                                        desc=self.cmd_PROBE_ACCURACY_help)
        
        self.gcode.register_mux_command('Z_OFFSET_APPLY_PROBE', 'PROBE_NAME',
                                        self.mcu_probe_name,
                                        self.cmd_Z_OFFSET_APPLY_PROBE,
                                        desc=self.cmd_Z_OFFSET_APPLY_PROBE_help)

def load_config_prefix(config):
    return ProbeG38multi(config)
