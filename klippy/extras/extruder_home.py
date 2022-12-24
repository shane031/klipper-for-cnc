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
import stepper, chelper, logging
from toolhead import Move
from collections import namedtuple

class ExtruderHoming:
    """
    ! WARNING EXPERIMENTAL UNTESTED
    This class registers a command to home an extruder's stepper.
    This is made possible due to a change in the Extruder class,
    which can now add an endstop to the extruder stepper, if a
    config parameter is provided.

    The "toolhead" passed to the PrinterHoming.manual_home method
    is not entirely "virtual". All methods are defined here, but
    some of them call the methods of the actual toolhead.
    """
    def __init__(self, config):
        self.printer = config.get_printer()
        self.extruder_name = config.get_name().split()[1]

        self.toolhead = None
        self.extruder = None
        self.gcmd = None
        self.th_orig_pos = None

        self.HOMING_DELAY = 0.001

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
    cmd_HOME_EXTRUDER_help = "Home an extruder using an endstop. The active extruder will be homed."
    def cmd_HOME_EXTRUDER(self, gcmd):
        
        # Get gcmd object, for later.
        self.gcmd = gcmd
        
        # NOTE: Get the toolhead and its extruder
        self.toolhead = self.printer.lookup_object("toolhead")
        self.extruder = self.toolhead.get_extruder()            # PrinterExtruder

        # NOTE: Get the steppers
        self.extruder_stepper = self.extruder.extruder_stepper  # ExtruderStepper
        self.stepper = self.extruder_stepper.stepper            # PrinterStepper
        self.steppers = [self.stepper]                          # [PrinterStepper]

        # NOTE: Get the "rail" from the extruder stepper.
        self.rail = self.extruder_stepper.rail                  # PrinterRail

        # NOTE: get the endstops from the extruder's PrinterRail.
        #       likely a list of tuples, each with an instance of 
        #       MCU_endstop and a stepper name.
        #       See PrinterRail at stepper.py.
        endstops = self.rail.get_endstops()                 # [(mcu_endstop, name)]
        
        # NOTE: get a PrinterHoming class from extras
        phoming = self.printer.lookup_object('homing')      # PrinterHoming

        # NOTE: Get toolhead position 
        self.th_orig_pos = self.toolhead.get_position()

        # NOTE: get homing information, speed and move coordinate.
        homing_info = self.rail.get_homing_info()
        speed = homing_info.speed
        pos = [0.0, 0.0, 0.0, None]
        pos[3] = self.get_movepos(homing_info)
        
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
        #           which is the "pos" list above:  pos = [0., 0., 0., 0.]
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

        # NOTE: Update positions in gcode_move, fixes inaccurate first
        #       relative move. Might not be needed since actually using 
        #       set_position from the TH.
        #       Might interfere with extruder move?
        # gcode_move = self.printer.lookup_object('gcode_move')
        # gcode_move.reset_last_position()

    def get_movepos(self, homing_info):
        # NOTE: based on "_home_axis" from CartKinematics, it estimates
        #       the distance to move for homing, at least for a G28 command.
        # Determine movement
        position_min, position_max = self.rail.get_range()  # 0, 100
        movepos = homing_info.position_endstop
        if homing_info.positive_dir:
            movepos -= 1.5 * (homing_info.position_endstop - position_min)  #   (0 - 0)*1.5 = 0
        else:
            movepos += 1.5 * (position_max - homing_info.position_endstop)  # (100 - 0)*1.5 = 150

        logging.info(f"\n\nget_movepos: movepos={str(movepos)}\n\n")
        return movepos

    def get_kinematics(self):
        """
        Virtual toolhead method.
        Called by:
            -   HomingMove.calc_toolhead_pos
            -   HomingMove.homing_move
        """
        # TEST: identical to manual_stepper, 
        #       methods for a "virtual kin" are defined here.
        return self
    
    def get_steppers(self):
        """
        Virtual toolhead method.
        Called by:
            -   HomingMove.calc_toolhead_pos,
            -   HomingMove.homing_move

        """
        # TEST: passes extruder stepper
        return self.steppers
    
    # TODO: Is this method from manual_stepper required?
    def sync_print_time(self):
        # NOTE: this function is likely making the "toolhead"
        #       wait for all moves to end before doing something
        #       else. Perhaps it will be called by "manual_home" below.
        #toolhead = self.printer.lookup_object('toolhead')
        print_time = self.toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            self.toolhead.dwell(self.next_cmd_time - print_time)
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

    def get_last_move_time(self):
        """
        Virtual toolhead method.
        Called by:
            -   HomingMove.homing_move
        """
        # TODO: What should I do here? Using toolhead method.
        # self.sync_print_time()
        # return self.next_cmd_time
        lmt = self.toolhead.get_last_move_time()
        logging.info(f"\n\nget_last_move_time: Last move time: {str(lmt)}\n\n")
        return lmt
    
    def dwell(self, delay):
        """
        Virtual toolhead method.
        Called by:
            -   HomingMove.homing_move
        """
        # TODO: What should I do here? Using toolhead method.
        #self.next_cmd_time += max(0., delay)
        # NOTE: once upon a time, there was no "drip_move" for homing.
        #       so a "dwell" was added to give an old RPi2 enough time
        #       to compute stuff:
        #       - https://github.com/Klipper3d/klipper/commit/78f4c25a14099564cf731bdaf5b97492a3a6fb47
        #       When the drip move was introduced, the dwell was significantly reduced:
        #       - https://github.com/Klipper3d/klipper/commit/dd34768e3afb6b5aa46885109182973d88df10b7
        #       Here the drip_move is _not_ used, and we thus require the
        #       extended dwell time, thereby ignoring the delay argument.

        # NOTE: pass the input
        # self.HOMING_DELAY = delay

        # NOTE: From homing.py
        # self.HOMING_DELAY = 0.001
        
        # NOTE: The original pre-drip value
        self.HOMING_DELAY = 0.250
        
        # NOTE: the 0.250 valued did not work at all,
        #       so it was increased by a nice amount,
        #       and now... It works! OMG :D Sometimes...
        # self.HOMING_DELAY = 4.0

        logging.info(f"\n\ndwell: Dwelling for {str(self.HOMING_DELAY)} before homing. Current print_time: {str(self.toolhead.print_time)}\n\n")
        self.toolhead.dwell(self.HOMING_DELAY)
        logging.info(f"\n\ndwell: Done sending dwell command. Current print_time: {str(self.toolhead.print_time)}\n\n")
    
    def move_extruder(self, newpos, speed, drip_completion):
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
        # self.do_move(newpos[0], speed, self.homing_accel)
        # TODO: What should I do here? What should the Move argument be?
        #       The "Extruder.move" method requires the following arguments:
        #       print_time: ???
        #       move: ???
        curpos = list(self.toolhead.commanded_pos)
        # move = Move(toolhead=self.toolhead, 
        #             start_pos=curpos,
        #             end_pos=curpos[:3] + [25.0],  # TODO: base this on the config
        #             speed=self.velocity)
        ntMove = namedtuple('Move', "axes_r accel start_v cruise_v axes_d accel_t cruise_t decel_t start_pos end_pos")
        move = ntMove(axes_r=[0,0,0,1.0], accel=20.0, start_v=0.0, cruise_v=20.0, axes_d=[None,None,None,25.0],
                        accel_t=1.0, cruise_t=5.0, decel_t=1.0, start_pos=0.0, end_pos=0.0)
        print_time = self.toolhead.print_time  # TODO: this should be OK if the dwell above is enough
        self.extruder.move(print_time=print_time, move=move)
        self.extruder.last_position = 0

        next_move_time = (print_time + move.accel_t + move.cruise_t + move.decel_t)

        self.toolhead._update_move_time(next_move_time)
        self.toolhead.last_kin_move_time = next_move_time

        pass
    
    def move_toolhead(self, newpos, speed, drip_completion):
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
        
        # NOTE: using "toolhead.manual_move" allows None values, is simpler
        #       to use in that sense, and also ends up calling "toolhead.move".
        # extra = 0.0
        # e_newpos = newpos[3] + extra
        # coord = [None, None, None, e_newpos]
        # logging.info(f"\n\nmove_toolhead: Moving {self.extruder.name} to {str(coord)} for homing.\n\n")  # Can be [None, None, None, 0.0]
        # self.toolhead.manual_move(coord=coord, speed=speed)

        # NOTE: using "toolhead.move" should be similar.
        logging.info(f"\n\nmove_toolhead: moving toolhead to {str(newpos)} for homing.\n\n")
        self.toolhead.move(newpos=newpos, speed=speed)
        logging.info(f"\n\nmove_toolhead: move completed.\n\n")

    def move_toolhead_drip(self, newpos, speed, drip_completion):
        """
        This method passes argument to the real toolhead "drip_move" method.
        """

        logging.info(f"\n\nmove_toolhead_drip: drip-moving to {str(newpos)} for homing.\n\n")  # Can be [None, None, None, 0.0]
        self.toolhead.drip_move(newpos, speed, drip_completion)

    def drip_move(self, newpos, speed, drip_completion):
        """
        Virtual toolhead method.
        Called by:
            -   HomingMove.homing_move
        """
        # NOTE: the "drip_completion" argument is an object that can be checked
        #       for to know if all of the endstops have been triggered yet.

        # NOTE: option 1, use the "manual_move" method from the ToolHead class.
        # TODO: Couldn't debug "Timer too close" nor "flush_handler" errors (at or after homing).
        #self.move_toolhead(newpos, speed, drip_completion)
        
        # NOTE: option 2, use the "move" method from the Extruder class.
        # TODO: I have not tried this, although I don't expect an improvement.
        self.move_extruder(newpos, speed, drip_completion)

        # NOTE: option 3, use the "drop move" method from the ToolHead class.
        # TODO: Couldn't debug "flush_handler" and some "Timer too close" errors only _after_ homing.
        # TODO: It's strange that the stepper stops at the endstop, and then moves a bit more... it shouldn't!
        # self.move_toolhead_drip(newpos, speed, drip_completion)

    def get_position(self):
        """
        Virtual toolhead method.
        Called by :
            -   _calc_endstop_rate
            -   calc_toolhead_pos
        """
        # TODO: What should I do here? Testing manual_stepper code directly.
        pos = [0., 0., 0., self.rail.get_commanded_position()]  # Can be [0.0, 0.0, 0.0, 0.0]
        logging.info(f"\n\nget_position output: {str(pos)}\n\n")
        return pos
    
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
        # NOTE: the set_position method in a toolhead calls set_position in a
        #       PrinterRail object, which we have here. That method calls 
        #       the set_position method in each of the steppers in the rail.
        # NOTE: At this point, this method receives a vector: "[4.67499999999994, 0.0, 0.0, 3.3249999999999402]"
        #       The first 3 items come from the "calc_position" method below.
        #       The first item is the "updated" position of the extruder stepper,
        #       corresponding to "haltpos", and the second and third are hard-coded 0s.
        #       The fourth component comes from "rail.get_commanded_position" here,
        #       which may correspond to the "old" extruder position, meaning that the
        #       rail has not been updated yet (sensibly because set_position is called later).
        #       This means that we need the original 123 toolhead components, and
        #       only update the remaining extruder component using the toolhead.set_position
        #       method. However it only sets the XYZ components in the XYZ "trapq".

        # TODO: I need to update the "E" queue somehow.
        #       When switching extruder steppers, the Extruder class uses the set_position
        #       method in its stepper object, passing it: [extruder.last_position, 0., 0.]
        #       Thus, I need a three element list, where the first element is the updated position.
        # logging.info(f"\n\nset_position input: {str(newpos)}\n\n")
        # coord = [newpos[0], 0.0, 0.0]
        # logging.info(f"\n\nset_position output: {str(coord)}\n\n")
        # self.rail.set_position(coord)

        # NOTE: setup "set_position" from the toolhead, this is expected for a complete
        #       regular homing move, and shouldnt hurt.
        logging.info(f"\n\nset_position: input={str(newpos)}\n\n")
        logging.info(f"\n\nset_position: old TH position={str(self.th_orig_pos)}\n\n")
        pos = self.th_orig_pos[:3] + [0]
        logging.info(f"\n\nset_position: output={str(pos)}\n\n")
        self.toolhead.set_position(pos)
        pass
    
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
        # NOTE: calc_toolhead_pos only uses the first three elements of this list,
        #       a fourth item  would be ignored.
        pos = [stepper_positions[self.rail.get_name()], 0., 0.]
        logging.info(f"\n\ncalc_position input: {str(stepper_positions)}\n\n")
        logging.info(f"\n\ncalc_position return: {str(pos)}\n\n")
        return pos

def load_config_prefix(config):
    return ExtruderHoming(config)

"""

get_movepos: movepos=150.0

get_last_move_time: Last move time: 29.0592399375

get_position output: [0.0, 0.0, 0.0, 0.0]

dwell: Dwelling for 4.0 before homing. Current last move time: 29.067680625


Stats 4736.1: gcodein=0  mcu: mcu_awake=0.002 mcu_task_avg=0.000032 mcu_task_stddev=0.000035 bytes_write=928 bytes_read=4203 bytes_retransmit=0 bytes_invalid=0 send_seq=115 receive_seq=115 retransmit_seq=0 srtt=0.004 rttvar=0.001 rto=0.025 ready_bytes=0 stalled_bytes=0 freq=15987465 tools: mcu_awake=0.007 mcu_task_avg=0.000073 mcu_task_stddev=0.000059 bytes_write=984 bytes_read=5374 bytes_retransmit=9 bytes_invalid=0 send_seq=110 receive_seq=110 retransmit_seq=2 srtt=0.004 rttvar=0.000 rto=0.025 ready_bytes=0 stalled_bytes=0 freq=15934599 adj=15947069  sysload=0.31 cputime=3.749 memavail=564980 print_time=33.069 buffer_time=3.718 print_stall=0 extruder: target=0 temp=169.7 pwm=0.000 extruder1: target=0 temp=169.9 pwm=0.000
Stats 4737.1: gcodein=0  mcu: mcu_awake=0.002 mcu_task_avg=0.000032 mcu_task_stddev=0.000035 bytes_write=934 bytes_read=4219 bytes_retransmit=0 bytes_invalid=0 send_seq=116 receive_seq=116 retransmit_seq=0 srtt=0.004 rttvar=0.001 rto=0.025 ready_bytes=0 stalled_bytes=0 freq=15987462 tools: mcu_awake=0.007 mcu_task_avg=0.000073 mcu_task_stddev=0.000059 bytes_write=1110 bytes_read=5692 bytes_retransmit=9 bytes_invalid=0 send_seq=121 receive_seq=121 retransmit_seq=2 srtt=0.004 rttvar=0.001 rto=0.025 ready_bytes=0 stalled_bytes=0 freq=15934588 adj=15947109  sysload=0.31 cputime=3.790 memavail=565032 print_time=33.069 buffer_time=2.718 print_stall=0 extruder: target=0 temp=170.3 pwm=0.000 extruder1: target=0 temp=170.5 pwm=0.000

dwell: Done sending dwell command. Current last move time: 33.0686685

move_toolhead: Moving extruder1 to [None, None, None, 150.0] for homing.

get_last_move_time: Last move time: 40.7706685

Stats 4738.1: gcodein=0  mcu: mcu_awake=0.002 mcu_task_avg=0.000032 mcu_task_stddev=0.000034 bytes_write=940 bytes_read=4248 bytes_retransmit=0 bytes_invalid=0 send_seq=117 receive_seq=117 retransmit_seq=0 srtt=0.004 rttvar=0.001 rto=0.025 ready_bytes=0 stalled_bytes=0 freq=15987461 tools: mcu_awake=0.007 mcu_task_avg=0.000073 mcu_task_stddev=0.000059 bytes_write=1708 bytes_read=6017 bytes_retransmit=9 bytes_invalid=0 send_seq=139 receive_seq=139 retransmit_seq=2 srtt=0.004 rttvar=0.000 rto=0.025 ready_bytes=0 stalled_bytes=0 freq=15934582 adj=15947013  sysload=0.31 cputime=3.862 memavail=565244 print_time=40.771 buffer_time=9.421 print_stall=0 extruder: target=0 temp=169.2 pwm=0.000 extruder1: target=0 temp=171.3 pwm=0.000
Stats 4739.1: gcodein=0  mcu: mcu_awake=0.002 mcu_task_avg=0.000032 mcu_task_stddev=0.000034 bytes_write=946 bytes_read=4264 bytes_retransmit=0 bytes_invalid=0 send_seq=118 receive_seq=118 retransmit_seq=0 srtt=0.004 rttvar=0.001 rto=0.025 ready_bytes=0 stalled_bytes=0 freq=15987456 tools: mcu_awake=0.018 mcu_task_avg=0.000116 mcu_task_stddev=0.000101 bytes_write=1834 bytes_read=6322 bytes_retransmit=9 bytes_invalid=0 send_seq=150 receive_seq=150 retransmit_seq=2 srtt=0.003 rttvar=0.000 rto=0.025 ready_bytes=0 stalled_bytes=0 freq=15934575 adj=15947070  sysload=0.31 cputime=3.894 memavail=565284 print_time=40.771 buffer_time=8.420 print_stall=0 extruder: target=0 temp=169.0 pwm=0.000 extruder1: target=0 temp=170.8 pwm=0.000
Stats 4740.1: gcodein=0  mcu: mcu_awake=0.002 mcu_task_avg=0.000032 mcu_task_stddev=0.000034 bytes_write=952 bytes_read=4280 bytes_retransmit=0 bytes_invalid=0 send_seq=119 receive_seq=119 retransmit_seq=0 srtt=0.004 rttvar=0.001 rto=0.025 ready_bytes=0 stalled_bytes=0 freq=15987453 tools: mcu_awake=0.018 mcu_task_avg=0.000116 mcu_task_stddev=0.000101 bytes_write=1960 bytes_read=6626 bytes_retransmit=9 bytes_invalid=0 send_seq=161 receive_seq=161 retransmit_seq=2 srtt=0.003 rttvar=0.000 rto=0.025 ready_bytes=0 stalled_bytes=0 freq=15934564 adj=15947067  sysload=0.37 cputime=3.928 memavail=565260 print_time=40.771 buffer_time=7.420 print_stall=0 extruder: target=0 temp=170.2 pwm=0.000 extruder1: target=0 temp=169.9 pwm=0.000

set_position input: [0.0, 0.0, 0.0, 150.0]

"""


""" Successfull drip home output: there seems to be a problem with the get_position / set position vectors

get_movepos: movepos=150.0

get_last_move_time: Last move time: 72.668525125

get_position output: [0.0, 0.0, 0.0, 0.0]

dwell: Dwelling for 0.001 before homing. Current last move time: 72.6729861875

dwell: Done sending dwell command. Current last move time: 72.67548725

get_last_move_time: Last move time: 72.922865625

set_position input: [0.0, 0.0, 0.0, 150.0]

get_position output: [0.0, 0.0, 0.0, 2.8849999999999496]

calc_position input: {'extruder1': 4.6599999999999495}

calc_position return: [4.6599999999999495, 0.0, 0.0]

set_position input: [4.6599999999999495, 0.0, 0.0, 2.8849999999999496]

"""

"""

# Movi el extruder "10 mm" hacia abajo después del reboot.

get_movepos: movepos=150.0  # Extruder home L93


get_last_move_time: Last move time: 75.023014  # homing.py L107


get_position output: [0.0, 0.0, 0.0, -10.000000000000153]   # homing.py L110 (call to _calc_endstop_rate).
                                                            # --> homing.py L66
                                                            # Los "10 mm" aparecen acá.

                                                            
dwell: Dwelling for 0.001 before homing. Current last move time: 75.0309443125  # homing.py L126
dwell: Done sending dwell command. Current last move time: 75.0348051875        # homing.py L126


# Drip move command happens here  # homing.py L134


get_last_move_time: Last move time: 75.824560125    # homing.py L142
                                                    # This is after sending the drip move.

                                                    
set_position input: [0.0, 0.0, 0.0, 150.0]  # homing.py L171
                                            # This is from the set to "movepos" line.


get_position output: [0.0, 0.0, 0.0, 3.709999999999932] # homing.py L174 (call to calc_toolhead_pos)
                                                        # --> homing.py L83

calc_position input: {'extruder1': 5.444999999999932}  # homing.py L174 (call to calc_toolhead_pos)
calc_position return: [5.444999999999932, 0.0, 0.0]    # --> homing.py L84


set_position input: [5.444999999999932, 0.0, 0.0, 3.709999999999932]    # homing.py L176
                                                                        # This is from the set to "haltpos" line.

"""