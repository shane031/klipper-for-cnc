# By naikymen and mdwasp
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
    """
    ! WARNING EXPERIMENTAL UNTESTED
    This class registers a command to home an extruder's stepper.
    This is made possible due to a change in the Extruder class,
    which can now add an endstop to the extruder stepper, if a
    config parameter is provided.
    """
    def __init__(self, config):
        self.printer = config.get_printer()
        self.extruder_name = config.get_name().split()[1]

        # NOTE: some parameters are loaded from the "extruder_homing" config section.
        self.velocity = config.getfloat('velocity', 5., above=0.)
        self.accel = self.homing_accel = config.getfloat('accel', 0., minval=0.)

        # TODO: find out what this is. The same is used in manual_stepper.py
        self.next_cmd_time = 0.
        
        # NOTE: The following command will become available with the syntax:
        #       "HOME_EXTRUDER EXTRUDER=extruder", the extruder name
        #        passed to the EXTRUDER argument might change.
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('HOME_EXTRUDER', "EXTRUDER",
                                   self.extruder_name, self.cmd_HOME_EXTRUDER,
                                   desc=self.cmd_HOME_EXTRUDER_help)
    
    # NOTE: a "do_enable" function is defined in manual_stepper.py
    #       It is unlikely that this is needed here, as the Extruder
    #       class may be managed in a different way.
    #       The same goes for "do_move"
    # TODO: consider defining "do_set_position" here as well.

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
        
        # NOTE: Get the toolhead and its extruder
        toolhead = self.printer.lookup_object("toolhead")
        self.extruder = toolhead.get_extruder()                 # PrinterExtruder
        
        # NOTE: Get the "rail" from the extruder stepper.
        self.rail = self.extruder_stepper.rail                  # PrinterRail

        # NOTE: Get the steppers
        self.extruder_stepper = self.extruder.extruder_stepper  # ExtruderStepper
        self.stepper = self.extruder_stepper.stepper            # PrinterStepper
        self.steppers = [self.stepper]                          # [PrinterStepper]

        pos = [0., 0., 0., 0.]
        speed = 5.0

        # TODO: Instantiate a new endstop instance from the
        #       new PrinterRail subclass: "RailFromStepper".
        endstops = self.rail.get_endstops()
        
        # NOTE: this loads the Homing class, from "extras/".
        phoming = self.printer.lookup_object('homing')
        
        phoming.manual_home(toolhead=self, endstops=endstops,
                            pos=pos, speed=speed,
                            triggered=True, 
                            check_trigger=True)
    
    # Toolhead wrappers to support homing
    def flush_step_generation(self):
        # TESTING: identical to manual_stepper
        self.sync_print_time()
    
    def get_position(self):
        # TODO: What should I do here?
        # return [self.rail.get_commanded_position(), 0., 0., 0.]
        pass
    def set_position(self, newpos, homing_axes=()):
        # TODO: What should I do here?
        # self.do_set_position(newpos[0])
        pass

    def get_last_move_time(self):
        # TESTING: identical to manual_stepper
        self.sync_print_time()
        return self.next_cmd_time
    
    def dwell(self, delay):
        # TESTING: identical to manual_stepper, may be irrelevant.
        self.next_cmd_time += max(0., delay)
        pass
    
    def drip_move(self, newpos, speed, drip_completion):
        # NOTE: some explanation on "drip" moves is available at:
        #       https://github.com/Klipper3d/klipper/commit/43064d197d6fd6bcc55217c5e9298d86bf4ecde7
        # NOTE: the manual_stepper class simply "moves" the stepper 
        #       in the regular way. However the ToolHead.drip_move does
        #       a lot more, in accordance with the commit linked above.
        # TODO: What should I do here?
        self.extruder.move()
        # self.do_move(newpos[0], speed, self.homing_accel)
        pass
    
    def get_kinematics(self):
        # TESTING: identical to manual_stepper
        return self
    
    def get_steppers():
        # TESTING: passes extruder stepper
        return self.steppers
    
    def calc_position(self, stepper_positions):
        # TODO: What should I do here?
        # NOTE: The get_name function is inherited from the
        #       first stepper in the steppers list of the
        #       PrinterRail class.
        # return [stepper_positions[self.rail.get_name()], 0., 0.]
        pass

def load_config_prefix(config):
    return ExtruderHoming(config)
