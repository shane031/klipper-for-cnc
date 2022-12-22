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

        self.toolhead = None

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
    
    # NOTE: the "register_mux_command" above registered a "HOME_EXTRUDER"
    #       command, which will end up calling this method.
    #       The "help" string is usually defined along the method.
    cmd_HOME_EXTRUDER_help = "Home an extruder using an endstop"
    def cmd_HOME_EXTRUDER(self, gcmd):
        
        # NOTE: Get the toolhead and its extruder
        self.toolhead = self.printer.lookup_object("toolhead")
        self.extruder = self.toolhead.get_extruder()            # PrinterExtruder

        # NOTE: Get the steppers
        self.extruder_stepper = self.extruder.extruder_stepper  # ExtruderStepper
        self.stepper = self.extruder_stepper.stepper            # PrinterStepper
        self.steppers = [self.stepper]                          # [PrinterStepper]

        # NOTE: Get the "rail" from the extruder stepper.
        self.rail = self.extruder_stepper.rail                  # PrinterRail

        pos = [0., 0., 0., 0.]
        speed = 5.0

        # NOTE: get the endstops from the extruder's PrinterRail.
        #       likely a list of tuples, each with an instance of 
        #       MCU_endstop and a stepper name.
        #       See PrinterRail at stepper.py.
        endstops = self.rail.get_endstops()                 # [(mcu_endstop, name)]
        
        # NOTE: get a PrinterHoming class from extras
        phoming = self.printer.lookup_object('homing')      # PrinterHoming
        
        # NOTE: "manual_home" is defined in the PrinterHoming class (at homing.py).
        #       The method instantiates a "HomingMove" class by passing it the
        #       "endstops" and "toolhead" objects. Here, the "self" object is passed
        #       as a "virtual toolhead", similar to what is done in manual_stepper.
        #       The provided endstops are from the extruder PrinterRail.
        # NOTE: "PrinterHoming.manual_home" then calls "HomingMove.homing_move".
        #       The "HomingMove" class downstream methods use the
        #       following methods from a provided "toolhead" object:
        #       - flush_step_generation
        #       - get_kinematics:           returning a "kin" object with methods:
        #           - kin.get_steppers:     returning a list of stepper objects.
        #           - kin.calc_position:    returning ???
        #       - get_position:             returning "thpos" (toolhead position)
        #       - get_last_move_time:       returning "print_time" (and later "move_end_print_time")
        #       - dwell
        #       - drip_move
        #       - set_position
        # NOTE: Other methods using the toolhead object or derivatives are also called:
        #       -   calc_toolhead_pos: This method receives a "movepos" argument,
        #           which is the "pos" list above.
        # NOTE: Of these methods, the Extruder class defines none.
        # NOTE: The object returned by "get_kinematics" is
        #       required to have the following methods:
        #       - get_steppers()
        #       - calc_position(kin_spos)
        # NOTE: The following command ends up calling the methods 
        #       in this class. For example "drip_move" for moving
        #       the extruder (towards the endstop, ideally).
        # NOTE: There are also other methods for homing:
        #       - probing_move ???
        #       - cmd_G28: ???
        # TODO: consider using those alternative methods.
        phoming.manual_home(toolhead=self, endstops=endstops,
                            pos=pos, speed=speed,
                            triggered=True, 
                            check_triggered=True)
    
    # TODO: Is this method from manual_stepper required?
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

    # Toolhead wrappers to support homing
    def flush_step_generation(self):
        """
        Virtual toolhead method.
        Called by:
            -   HomingMove.homing_move
        """
        # TODO: What should I do here?
        #       I think it is allright to use the toolheads method,
        #       because the extruder stepper is part of the toolhead.
        #       Also, the alternative "sync_print_time" does not make
        #       much sense to me, because it was used to get the latest
        #       print time, to be used in other manual_stepper methods.
        #self.sync_print_time()
        self.toolhead.flush_step_generation()
    
    def get_position(self):
        """
        Virtual toolhead method.
        Called by 
        """
        # TODO: What should I do here? Testing manual_stepper code directly.
        return [self.rail.get_commanded_position(), 0., 0., 0.]
    
    def set_position(self, newpos, homing_axes=()):
        """
        Virtual toolhead method.
        Called by:
            -   HomingMove.homing_move
        """
        # TODO: What should I do here?
        # NOTE: I am assuming that the "set_position" applies to steppers,
        #       by tracing calls to the "set_position" method in MCU_stepper.
        #       There, the "coords" argument is a list of at least 3 components:
        #           [coord[0], coord[1], coord[2]]
        #       I do not know why it needs three components for a steper object,
        #       but from the "itersolve_set_position" code, they seem to be x,y,z 
        #       components.
        # self.do_set_position(newpos[0])
        pass

    def get_last_move_time(self):
        """
        Virtual toolhead method.
        Called by:
            -   HomingMove.homing_move
        """
        # TODO: What should I do here? Testing manual_stepper code directly.
        self.sync_print_time()
        return self.next_cmd_time
    
    def dwell(self, delay):
        """
        Virtual toolhead method.
        Called by:
            -   HomingMove.homing_move
        """
        # TODO: What should I do here?
        #self.next_cmd_time += max(0., delay)
        self.toolhead.dwell(delay)
    
    def drip_move_extruder(self, newpos, speed, drip_completion):
        """
        This method is an alternative to implement "drip_move",
        using the Extruder.move command. To use it instead of other
        methods in this class, uncomment it in the "drip_move" method,
        and comment the others.
        """
        # NOTE: Explanation of drip_move arguments:
        #       - newpos: new position vector [0,0,0,0]
        #       - speed:  ???
        #       - drip_completion: ???
        # NOTE: some explanation on "drip" moves is available at:
        #       https://github.com/Klipper3d/klipper/commit/43064d197d6fd6bcc55217c5e9298d86bf4ecde7
        # NOTE: the manual_stepper class simply "moves" the stepper 
        #       in the regular way. However the ToolHead.drip_move does
        #       a lot more, in accordance with the commit linked above.
        # TODO: What should I do here?
        #       The "Extruder.move" method requires the following arguments:
        #       print_time: ???
        #       move: ???
        self.extruder.move()
        # self.do_move(newpos[0], speed, self.homing_accel)

    
    def drip_move_toolhead(self, newpos, speed, drip_completion):
        """
        This method is an alternative to implement "drip_move",
        using the ToolHead.manual_move command. To use it instead of other
        methods in this class, uncomment it in the "drip_move" method,
        and comment the others.
        """
        # NOTE: Explanation of drip_move arguments:
        #       - newpos: new position vector [0,0,0,0]
        #       - speed:  ???
        #       - drip_completion: ???
        # NOTE: The manual_move method allows "None" values to be passed,
        #       allowing me not to worry about getting the current and new
        #       coordinates for the homing move.
        self.toolhead.manual_move(coord=[None, None, None, newpos[3]],
                                  speed=speed)
    
    def drip_move(self, newpos, speed, drip_completion):
        """
        Virtual toolhead method.
        Called by:
            -   HomingMove.homing_move
        """
        # NOTE: option 1, use the "manual_move" method from the ToolHead class.
        self.drip_move_toolhead(self, newpos, speed, drip_completion)
        
        # NOTE: option 2, use the "move" method from the Extruder class.
        #self.drip_move_extruder(self, newpos, speed, drip_completion)
    
    def get_kinematics(self):
        """
        Virtual toolhead method.
        Called by:
            -   HomingMove.calc_toolhead_pos
            -   HomingMove.homing_move
        """
        # TESTING: identical to manual_stepper
        return self
    
    def get_steppers(self):
        """
        Virtual toolhead method.
        Called by:
            -   HomingMove.calc_toolhead_pos,
            -   HomingMove.homing_move

        """
        # TESTING: passes extruder stepper
        return self.steppers
    
    def calc_position(self, stepper_positions):
        """
        Virtual toolhead method.
        Called by HomingMove.calc_toolhead_pos
        """
        # TODO: What should I do here? Testing manual_stepper code directly.
        #       This is also very similar to the CartKinematics method.
        # NOTE: The get_name function is inherited from the
        #       first stepper in the steppers list of the
        #       PrinterRail class.
        return [stepper_positions[self.rail.get_name()], 0., 0.]

def load_config_prefix(config):
    return ExtruderHoming(config)
