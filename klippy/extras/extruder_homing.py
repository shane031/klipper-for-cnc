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

Q: where does this code go? A manual stepper is defined in "extras", perhaps loaded by this code: https://github.com/Klipper3d/klipper/blob/b026f1d2c975604a0ea7ff939f4c36ef3df80a41/klippy/klippy.py#L108

It's meant to go in a new file in extras.
You'll have to instantiate a new Endstop instance somewhere (to replace [mcu_endstop]).
"""
import stepper, chelper

class ExtruderHoming:
    def __init__(self, config):
        self.printer = config.get_printer()

        # NOTE: get the extruder name from the config heading: "[extruder_homing NAME_HERE]".
        extruder_name = config.get_name().split()[1]
        
        # NOTE: get the extruder object by the specified name using "lookup_object".
        #       This means that a corresponding extruder must be in the config.
        #       The extruder section always looks like "[extruder]" with no secondary name.
        #       This probably means that "extruder_name" will always be the same.
        # NOTE: The "lookup_object" method is from the Printer class (defined in klippy.py),
        #       it uses the full name to get objects (i.e. "extruder_stepper hola").
        self.extruder = self.printer.lookup_object(extruder_name)

        # NOTE: some parameters are loaded from the "extruder_homing" config section.
        self.velocity = config.getfloat('velocity', 5., above=0.)
        self.accel = self.homing_accel = config.getfloat('accel', 0., minval=0.)

        # TODO: find out what this is.
        self.next_cmd_time = 0.

        # TODO: I added a rail definition from manual_stepper#L14
        #       What does PrinterRail read from "config"?
        self.rail = stepper.PrinterRail(
            config, need_position_minmax=False, default_position_endstop=0.)
        
        # Register commands
        # NOTE: this command will become available with the syntax:
        #       "HOME_EXTRUDER EXTRUDER=extruder", the extruder name
        #        passed to the EXTRUDER argument might change.
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('HOME_EXTRUDER', "EXTRUDER",
                                   extruder_name, self.cmd_HOME_EXTRUDER,
                                   desc=self.cmd_HOME_EXTRUDER_help)
    
    def sync_print_time(self):
        # NOTE: this function is likely making the "toolhead"
        #       wait for all moves to end before doing something
        #       else. Perhaps it will be called by "manual_home" below.
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time
    
    # NOTE: the "register_mux_command" above registered a "HOME_EXTRUDER"
    #       command, which will end up calling this method.
    #       The "help" string is usually defined along the method.
    cmd_HOME_EXTRUDER_help = "Home an extruder using an endstop"
    def cmd_HOME_EXTRUDER(self, gcmd):
        self.homing_accel = accel
        pos = [0., 0., 0., 0.]

        # TODO: Instantiate a new endstop instance
        #       I tried something. Is this correct?
        #       I did it as shown in manual_stepper.py#L81
        endstops = self.rail.get_endstops()
        
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
