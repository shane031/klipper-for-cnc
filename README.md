Welcome to the Klipper project!

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

https://www.klipper3d.org/

Klipper is a 3d-Printer firmware. It combines the power of a general
purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more
information on why you should use Klipper.

To begin using Klipper start by
[installing](https://www.klipper3d.org/Installation.html) it.

Klipper is Free Software. See the [license](COPYING) or read the
[documentation](https://www.klipper3d.org/Overview.html). We depend on
the generous support from our
[sponsors](https://www.klipper3d.org/Sponsors.html).

# Fork notes

This fork implements:

- Homing on the steppers of `[extruder]`s.
    - Module: [extruder_home.py](./klippy/extras/extruder_home.py)
    - Command: `HOME_EXTRUDER EXTRUDER=extruder`.
    - Caveats: `extruder` must be active (use [this](https://github.com/naikymen/klipper-homing-extruder/blob/pipetting/config/configs-pipetting-bot/config-pi-pico-mainsail/home_extruder.cfg#L21) macro for convenience). No "second home" is performed. It probably won't work on extruder steppers configured as `[extruder_stepper]` later synced to an `[extruder]`.
- Probing in arbitrary directions with `G38.2`, `G38.3`, `G38.4`, and `G38.5`.
    - Module: [probe_G38.py](./klippy/extras/probe_G38.py)
    - Example command: `G38.2 X20 F10`
    - Notes: affected by `G90`/`G91`.
    - For reference, see [LinuxCNC](http://linuxcnc.org/docs/stable/html/gcode/g-code.html#gcode:g38)'s definition of _G38.n Straight Probe_ commands.
- General probing with multiple pins is supported by an experimental module:
    - Module: [probe_G38_multi.py](./klippy/extras/probe_G38_multi.py)
    - Example command: `MULTIPROBE2 PROBE_NAME=p20 Z=-20 F=1` (replace `2` by `3-5` for the other probing modes).
    - Notes: affected by `G90`/`G91`.

Minor modifications in Klippy's core were made to accommodate these features.

## Interested in CNC stuff for Klipper?

Let's chat over here: https://klipper.discourse.group/t/klipper-forks-for-cnc/5698

Cheers!

## Configs

See examples here: [config-pi-pico-mainsail](./config/configs-pipetting-bot/config-pi-pico-mainsail)

### Minimal homing config

Main config: [printer.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap/printer.cfg)

```yaml
[extruder]
# ...
# ...
# Setup all the usual endstop parameters before these:
position_endstop: 0.0
position_min: 0.0
position_max: 100.0
homing_speed: 25.0
second_homing_speed: 25.0
homing_retract_speed: 25.0
homing_retract_dist: 2.0
homing_positive_dir: False
endstop_pin: gpio15

[extruder_home extruder]
velocity: 25.0
accel: 100.0

[extruder1]
# ...
# ...
# Setup all the usual endstop parameters before these:
position_endstop: 0.0
position_min: 0.0
position_max: 100.0
homing_speed: 25.0
second_homing_speed: 25.0
homing_retract_speed: 25.0
homing_retract_dist: 2.0
homing_positive_dir: False
endstop_pin: gpio18

[extruder_home extruder1]
velocity: 25.0
accel: 100.0
```

### Minimal probing config

Config: [probe_G38.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap/probe_G38.cfg)

```yaml
[probe_G38]
recovery_time: 0.4
pin: gpio19
z_offset: 0
```

### Minimal multi-probing config

Config: [probe_G38_multi.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap/probe_G38_multi.cfg)

```yaml
# CNC shield v3.0 pin map:
# https://gitlab.com/pipettin-bot/pipettin-grbl/-/blob/master/doc/electronica/arduino_cnc_shield/klipper_pin_map.svg

[probe_G38_multi p20]
# Activate the "probe_G38.py" extras module, providing generalized G38.2 probing.
#
# Settings borrowed from "[smart_effector]".
recovery_time: 0.0
# Settings from original "[probe]" section.
pin: ^tools:PB4
z_offset: 0


[probe_G38_multi p200]
# Activate the "probe_G38.py" extras module, providing generalized G38.2 probing.
#
# Settings borrowed from "[smart_effector]".
recovery_time: 0.0
# Settings from original "[probe]" section.
pin: ^tools:PC0
z_offset: 0
```