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
        #       "NAME_HERE" can be "extruder", "extruder1", etc. (see discussion below).
        extruder_name = config.get_name().split()[1]
        
        # NOTE: get the extruder object by the specified name using "lookup_object".
        #       This means that a corresponding extruder must be in the config.
        #       The extruder section always looks like "[extruder]" with no secondary name.
        #       This probably means that "extruder_name" will always be the same: "extruder"
        #       In fact, setting a name causes this error:
        #           "Section 'extruder sarasa' is not a valid config section"
        #       It seems, however, that extra extruders can be defined with no spaces: "[extruder1]"
        #       See: https://github.com/Klipper3d/klipper/blob/master/config/sample-multi-extruder.cfg
        # NOTE: The "lookup_object" method is from the Printer class (defined in klippy.py),
        #       it uses the full name to get objects (i.e. "extruder_stepper hola").
        self.extruder = self.printer.lookup_object(extruder_name)
        self.extruder_stepper = self.extruder.extruder_stepper
        self.stepper = self.extruder_stepper.stepper
        self.steppers = [self.stepper]
        # NOTE: it is likely that a "SYNC_EXTRUDER_MOTION" has to be issued
        #       in order to home the different extruder/tool steppers.
        #       Unless a multi-extruder config is used (insetead of multiple extruder_steppers).
        #       In that case, the "ACTIVATE_EXTRUDER" command must be used.

        # NOTE: some parameters are loaded from the "extruder_homing" config section.
        self.velocity = config.getfloat('velocity', 5., above=0.)
        self.accel = self.homing_accel = config.getfloat('accel', 0., minval=0.)

        # TODO: find out what this is. The same is used in manual_stepper.py
        self.next_cmd_time = 0.

        # NOTE: there is a code section in manual_stepper.py about
        #       "# Setup iterative solver", working with "trapq" stuff,
        #       probably "C" code bindings.
        #       Then the result of this setup is passed to the manual
        #       stepper rail definition.
        # NOTE: "trapq" probably means "trapezoid motion queue",
        #       as stated in "toolhead.py".

        # TODO: I added a rail definition from manual_stepper#L14
        #       which uses PrinterRail as defined in manual_stepper.py
        #       Is this the way to go? Shouldn't I use the extruder stepper?
        #self.rail = stepper.PrinterRail(config, 
        #                                need_position_minmax=False,
        #                                default_position_endstop=0.)
        
        # TODO: manual_stepper also loads the steppers from the rail
        #self.steppers = self.rail.get_steppers() 
        
        # NOTE: The following command will become available with the syntax:
        #       "HOME_EXTRUDER EXTRUDER=extruder", the extruder name
        #        passed to the EXTRUDER argument might change.
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('HOME_EXTRUDER', "EXTRUDER",
                                   extruder_name, self.cmd_HOME_EXTRUDER,
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

    # NOTE: defining the do_homing_move method from manual_stepper
    #       at least for reference.
    def do_homing_move(self, movepos, speed, accel, triggered, check_trigger):
        
        # NOTE: this does not apply.
        # if not self.can_home:
        #     raise self.printer.command_error(
        #         "No endstop for this manual stepper")
        
        self.homing_accel = accel
        pos = [movepos, 0., 0., 0.]
        endstops = self.rail.get_endstops()
        phoming = self.printer.lookup_object('homing')
        phoming.manual_home(toolhead=self, endstops=endstops,
                            pos=pos, speed=speed,
                            triggered=triggered, 
                            check_trigger=check_trigger)
    
    # NOTE: the "register_mux_command" above registered a "HOME_EXTRUDER"
    #       command, which will end up calling this method.
    #       The "help" string is usually defined along the method.
    cmd_HOME_EXTRUDER_help = "Home an extruder using an endstop"
    def cmd_HOME_EXTRUDER(self, gcmd):
        self.homing_accel = accel
        # NOTE: manual_stepper uses "[movepos, 0., 0., 0.]" instead.
        #       The "movepos" is provided by the MOVE argument to the
        #       MANUAL_STEPPER command, and indicates the position to
        #       which the stepper will move. If it reaches an endstop,
        #       it can stop before reaching "movepos".
        # TODO: What "movepos" should I use for this stepper?
        #       Why was "0" suggested?
        pos = [0., 0., 0., 0.]

        # TODO: Instantiate a new endstop instance
        #       I tried something. Is this correct?
        #       I did it as shown in manual_stepper.py#L81
        #endstops = self.rail.get_endstops()
        
        phoming = self.printer.lookup_object('homing')
        # NOTE: "manual_home" defined in the PrinterHoming class (at homing.py).
        #       The method instantiates a "HomingMove" class by passing it the
        #       "endstops" and "toolhead" objects.
        #       It then uses the "HomingMove.homing_move" method.
        # NOTE: Here it is important that "self", which is passed as the "toolhead"
        #       argument, has a "get_kinematics" method, which returns the appropriate
        #       kinematics object; probably from an "Extruder" class.
        # TODO: the HomingMove class uses the following methods from a "toolhead" object:
        #       - get_position
        #       - get_kinematics
        #       - flush_step_generation
        #       - get_last_move_time
        #       - dwell
        #       - drip_move
        #       - set_position
        # NOTE: Of these methods, the Extruder class defines nonte
        # TODO: The object returned by "get_kinematics" is
        #       required to have the following methods:
        #       - get_steppers()
        #       - calc_position(kin_spos)
        # TODO: All of these are set to "pass" below. Should they?
        #       The manual_stepper methods actually do some stuff.
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
        # return [stepper_positions[self.rail.get_name()], 0., 0.]
        pass

def load_config_prefix(config):
    return ExtruderHoming(config)
