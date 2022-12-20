# By mdwasp at Discord
# https://discord.com/channels/627982902738681876/1046618170993160202/1046808809894588457
"""
look at the chain of ManualStepper via PrinterRail
    Ref: https://github.com/Klipper3d/klipper/blob/cc63fd51b226f86aa684342241c20e2e2b84f8da/klippy/extras/manual_stepper.py#L14
then there's some helper methods to emulate a toolhead in ManualStepper
    Ref: https://github.com/Klipper3d/klipper/blob/cc63fd51b226f86aa684342241c20e2e2b84f8da/klippy/extras/manual_stepper.py#L106
that virtual "toolhead" is passed into the homing move
    Ref: https://github.com/Klipper3d/klipper/blob/cc63fd51b226f86aa684342241c20e2e2b84f8da/klippy/extras/manual_stepper.py#L83
you would need to write a component like "ExtruderHoming" that wraps the original Extruder
    Extruder kinmatics: https://github.com/Klipper3d/klipper/blob/master/klippy/kinematics/extruder.py
implement the necessary methods to be a toolhead object for homing moves
and then register a custom homing command
maybe 30 lines of python

Q: where does this code go?
A manual stepper is defined in "extras", perhaps loaded by this code: https://github.com/Klipper3d/klipper/blob/b026f1d2c975604a0ea7ff939f4c36ef3df80a41/klippy/klippy.py#L108
"""
import stepper, chelper

class ExtruderHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        extruder_name = config.get_name().split()[1]
        self.extruder = self.printer.lookup_object(extruder_name)
        self.velocity = config.getfloat('velocity', 5., above=0.)
        self.accel = self.homing_accel = config.getfloat('accel', 0., minval=0.)
        self.next_cmd_time = 0.
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('HOME_EXTRUDER', "EXTRUDER",
                                   extruder_name, self.cmd_HOME_EXTRUDER,
                                   desc=self.cmd_HOME_EXTRUDER_help)
    def sync_print_time(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time
    cmd_HOME_EXTRUDER_help = "Home an extruder using an endstop"
    def cmd_HOME_EXTRUDER(self, gcmd):
        self.homing_accel = accel
        pos = [0., 0., 0., 0.]
        endstops = [mcu_endstop]
        phoming = self.printer.lookup_object('homing')
        phoming.manual_home(self, endstops, pos, speed, True, True)
    # Toolhead wrappers to support homing
    def flush_step_generation(self):
        pass
    def get_position(self):
        pass
    def set_position(self, newpos, homing_axes=()):
        pass
    def get_last_move_time(self):
        pass
    def dwell(self, delay):
        pass
    def drip_move(self, newpos, speed, drip_completion):
        pass
    def get_kinematics(self):
        pass

def load_config_prefix(config):
    return ExtruderHoming(config)
