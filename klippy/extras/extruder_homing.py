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

# TODO: trying to subclass PrinterRail, in order to provide the extruder
#       stepper directly, instead of it creating a new one from a config.
class RailFromStepper(stepper.PrinterRail):
    """
    PrinterRail subclass with a few modifications.
    The add_extra_stepper method was modified to accept a stepper
    argument, and use it instead of creating a new one from configs.
    The idea is to use this class to add an endstop to an
    extruder stepper.
    """
    def __init__(self, config, stepper, 
                 need_position_minmax=True,
                 default_position_endstop=None,
                 units_in_radians=False):
        # Primary stepper and endstop
        self.stepper_units_in_radians = units_in_radians
        self.steppers = []
        self.endstops = []
        self.endstop_map = {}

        # NOTE: Reuse a "stepper" object from an Extruder,
        #       and add it to the "self.steppers" list.
        #       add_extra_stepper then handles the "setup" of the associated
        #       endstop into an MCU_endstop class, and also adds
        #       the stepper to this class.
        # TODO:  In the original class, PrinterStepper function instantiates an
        #       "MCU_stepper" class, registers it in several modules,
        #       and returns it. This is omitted now.
        self.add_extra_stepper(config, stepper)
        
        # NOTE: this grabs the first "MCU_stepper" item in the list.
        mcu_stepper = self.steppers[0]
        # NOTE: The get_name function is inherited from the
        #       first stepper in the steppers list of the
        #       PrinterRail class. It thus keeps only the first
        #       one. I imagine something like this:
        #       Keep "stepper_x" from ["stepper_x", "stepper_x1"]
        self.get_name = mcu_stepper.get_name
        # TODO: I don't know what these do yet.
        self.get_commanded_position = mcu_stepper.get_commanded_position
        self.calc_position_from_coord = mcu_stepper.calc_position_from_coord
        
        # Primary endstop position
        mcu_endstop = self.endstops[0][0]
        if hasattr(mcu_endstop, "get_position_endstop"):
            self.position_endstop = mcu_endstop.get_position_endstop()
        elif default_position_endstop is None:
            self.position_endstop = config.getfloat('position_endstop')
        else:
            self.position_endstop = config.getfloat(
                'position_endstop', default_position_endstop)
        # Axis range
        if need_position_minmax:
            self.position_min = config.getfloat('position_min', 0.)
            self.position_max = config.getfloat(
                'position_max', above=self.position_min)
        else:
            self.position_min = 0.
            self.position_max = self.position_endstop
        if (self.position_endstop < self.position_min
            or self.position_endstop > self.position_max):
            raise config.error(
                "position_endstop in section '%s' must be between"
                " position_min and position_max" % config.get_name())
        # Homing mechanics
        self.homing_speed = config.getfloat('homing_speed', 5.0, above=0.)
        self.second_homing_speed = config.getfloat(
            'second_homing_speed', self.homing_speed/2., above=0.)
        self.homing_retract_speed = config.getfloat(
            'homing_retract_speed', self.homing_speed, above=0.)
        self.homing_retract_dist = config.getfloat(
            'homing_retract_dist', 5., minval=0.)
        self.homing_positive_dir = config.getboolean(
            'homing_positive_dir', None)
        if self.homing_positive_dir is None:
            axis_len = self.position_max - self.position_min
            if self.position_endstop <= self.position_min + axis_len / 4.:
                self.homing_positive_dir = False
            elif self.position_endstop >= self.position_max - axis_len / 4.:
                self.homing_positive_dir = True
            else:
                raise config.error(
                    "Unable to infer homing_positive_dir in section '%s'"
                    % (config.get_name(),))
            config.getboolean('homing_positive_dir', self.homing_positive_dir)
        elif ((self.homing_positive_dir
               and self.position_endstop == self.position_min)
              or (not self.homing_positive_dir
                  and self.position_endstop == self.position_max)):
            raise config.error(
                "Invalid homing_positive_dir / position_endstop in '%s'"
                % (config.get_name(),))

    # NOTE: overriding the default "add_extra_stepper" here.
    def add_extra_stepper(self, config, stepper):
        """
        This method overrides the original PrinterRail method.
        The aim is to avoid creating a stepper, and replace that
        by a stepper object provided in the arguments.
        The idea is to add an endstop to an extruder stepper.
        """
        
        # NOTE: for the record...
        #stepper = PrinterStepper(config, self.stepper_units_in_radians)

        # NOTE: the "stepper" argument is supposed to be an extruder stepper.
        self.steppers.append(stepper)

        # NOTE: Check if self.endstops has been populated, initially its empty "[]".
        #       If and endstop has been added, and no 'endstop_pin' was defined
        #       in the config, then "use the primary endstop", and return immediately.
        # NOTE: It should not be the case for the use of this subclass.
        if self.endstops and config.get('endstop_pin', None) is None:
            # No endstop defined - use primary endstop
            self.endstops[0][0].add_stepper(stepper)
            return
        
        endstop_pin = config.get('endstop_pin')
        printer = config.get_printer()

        # NOTE: Get object from pins.py
        ppins = printer.lookup_object('pins')

        # NOTE: calls a PrinterPins method from pins.py,
        #       which does "Pin to chip mapping".
        #       It returns a dict with some properties.
        pin_params = ppins.parse_pin(endstop_pin,
                                     can_invert=True,
                                     can_pullup=True)
        
        # Normalize pin name
        pin_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])
        
        
        # NOTE: get() method from dict:
        #       "Return the value for key if key is in the dictionary, else default."
        endstop = self.endstop_map.get(pin_name, default=None)
        
        # Look for already-registered endstop
        if endstop is None:
            # New endstop, register it
            
            # NOTE: I don't really get what this does yet.
            #       It uses "lookup_pin" which registers an active pin.
            #       It also calls the "setup_pin" method on a "chip" object. 
            #       The chip object comes from a call to "register_chip" elsewhere.
            #       In mcu.py, the MCU class passes itself to this method. 
            #       So... the chips may be MCUs.
            # NOTE: as commented in pins.py (L136), mcu_endstop is 
            #       likely an instance of the MCU_endstop class,
            #       as defined in "mcu.py".
            mcu_endstop = ppins.setup_pin(pin_type='endstop', pin_desc=endstop_pin)
            
            # NOTE: I don't really get what this does yet.
            #       Add the endstop to the "endstop_map" class dict.
            self.endstop_map[pin_name] = {'endstop': mcu_endstop,
                                          'invert': pin_params['invert'],
                                          'pullup': pin_params['pullup']}
            
            # NOTE: I don't really get what this does yet.
            #       Add the endstop to the "endstops" class list.
            # NOTE: as commented above, mcu_endstop is likely
            #       an instance of the MCU_endstop class, as
            #       defined in "mcu.py".
            name = stepper.get_name(short=True)
            self.endstops.append((mcu_endstop, name))
            
            # Load the "query_endstops" module.
            query_endstops = printer.load_object(config, 'query_endstops')
            # Register the endstop there.
            query_endstops.register_endstop(mcu_endstop, name)
        else:
            # endstop already registered
            # NOTE: check if the invert or pull-up pins were
            #       configured differently, and raise an error
            #       if they are.
            mcu_endstop = endstop['endstop']
            changed_invert = pin_params['invert'] != endstop['invert']
            changed_pullup = pin_params['pullup'] != endstop['pullup']
            if changed_invert or changed_pullup:
                raise error("Pinter rail %s shared endstop pin %s "
                            "must specify the same pullup/invert settings" % (
                                self.get_name(), pin_name))
        # NOTE: call the "add_stepper" method from the
        #       MCU_endstop class, which in turn calls
        #       the "trsync.add_stepper" method from the
        #       MCU_trsync class, which simply appends
        #       "stepper" object to a list of steppers.
        mcu_endstop.add_stepper(stepper)



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

        # Create a "rail" from the extruder stepper.
        self.rail = RailFromStepper(config=config, stepper=self.stepper,
                                    need_position_minmax=False,
                                    default_position_endstop=0.)

        # TODO: with a rail defined, i could try using 
        #       the "Homing.home_rails" class/method instead.
        #       Not sure about the difference.

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
    
    # NOTE: the "register_mux_command" above registered a "HOME_EXTRUDER"
    #       command, which will end up calling this method.
    #       The "help" string is usually defined along the method.
    cmd_HOME_EXTRUDER_help = "Home an extruder using an endstop"
    def cmd_HOME_EXTRUDER(self, gcmd):
        
        # TODO: check why mdwasp wrote this, since it is defined above,
        #       and also not really used elsewhere in the code.
        #self.homing_accel = accel

        # NOTE: manual_stepper uses "[movepos, 0., 0., 0.]" instead.
        #       The "movepos" is provided by the MOVE argument to the
        #       MANUAL_STEPPER command, and indicates the position to
        #       which the stepper will move. If it reaches an endstop,
        #       it can stop before reaching "movepos".
        # TODO: What "movepos" should I use for this stepper?
        #       Why was "0" suggested?
        # TODO: try replacing this with the value from the config file.
        #       RailFromStepper requires lots of homing parameters anyway,
        #       including: "homing_speed", "homing_retract_dist",
        #       "homing_positive_dir", and others.
        pos = [0., 0., 0., 0.]

        # TODO: Instantiate a new endstop instance from the
        #       new PrinterRail subclass: "RailFromStepper".
        endstops = self.rail.get_endstops()
        
        # NOTE: this loads the Homing class, from "extras/".
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
        # NOTE: The object returned by "get_kinematics" is
        #       required to have the following methods:
        #       - get_steppers()
        #       - calc_position(kin_spos)
        # NOTE: The following command ends up calling the methods 
        #       in this class. For example "drip_move" for moving
        #       the extruder (towards the endstop, ideally).
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
