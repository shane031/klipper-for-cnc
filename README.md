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
- Probing in arbitrary directions with `G38.2`, `G38.3`, `G38.4`, and `G38.5` (single-probe version).
    - Module: [probe_G38.py](./klippy/extras/probe_G38.py)
    - Example command: `G38.2 X20 F10`
    - For reference, see [LinuxCNC](http://linuxcnc.org/docs/stable/html/gcode/g-code.html#gcode:g38)'s definition of _G38.n Straight Probe_ commands.
    - Note: affected by `G90`/`G91` and `M82`/ `M83`.
    - Known incompatibilites: `[probe_G38_multi ...]`
- General probing with multiple probe pins is supported by an experimental module:
    - Module: [probe_G38_multi.py](./klippy/extras/probe_G38_multi.py)
    - Example multi-command: `MULTIPROBE2 PROBE_NAME=extruder1 Z=-20 F=1` (replace `2` by `3-5` for the other probing modes).
    - Example mono-command: `G38.2 X20 F10` (replace `.2` by `.3-.5` for the other probing modes). To choose the probe pin, this command will try to match the probe's config name to an extruder name, or fail.
    - Note: affected by `G90`/`G91` and `M82`/ `M83`.
    - Known incompatibilites: `[probe_G38]`

Minor modifications in Klippy's core were made to accommodate these features.

## Interested in CNC stuff for Klipper?

Let's chat over here: https://klipper.discourse.group/t/klipper-forks-for-cnc/5698

Cheers!

## Configs

See examples here: [config-pi-pico-mainsail](./config/configs-pipetting-bot/config-pi-pico-mainsail)

These are meant as reference configs; you _must_ adjust them to match your setup first.

### Extruder homing config

Main config: [printer.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap/printer.cfg)

At a glance:

```yaml
[extruder]
# ...
# ...
# Setup all the usual extruder parameters before these:
position_endstop: 0.0
position_min: 0.0
position_max: 100.0
homing_speed: 25.0
second_homing_speed: 25.0
homing_retract_speed: 25.0
homing_retract_dist: 2.0
homing_positive_dir: False  # ADJUST TO MATCH YOUR SETUP
endstop_pin: gpio15  # REPLACE WITH THE PIN OF **YOUR** HOMING ENDSTOP

[extruder_home extruder]
# No parameters needed.

[extruder1]
# ...
# ...
# Setup all the usual extruder parameters before these.
position_endstop: 0.0
position_min: 0.0
position_max: 100.0
homing_speed: 25.0
second_homing_speed: 25.0
homing_retract_speed: 25.0
homing_retract_dist: 2.0
homing_positive_dir: False  # ADJUST TO MATCH YOUR SETUP
endstop_pin: gpio18  # REPLACE WITH THE PIN OF **YOUR** HOMING ENDSTOP

[extruder_home extruder1]
# No parameters needed.
```

### Probing config

Config: [probe_G38.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap/probe_G38.cfg)

```yaml
[probe_G38]
recovery_time: 0.4
pin: gpio19
z_offset: 0
```

### Multi-probing config

Config: [probe_G38_multi.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap/probe_G38_multi.cfg)

```yaml
# CNC shield v3.0 pin map:
# https://gitlab.com/pipettin-bot/pipettin-grbl/-/blob/master/doc/electronica/arduino_cnc_shield/klipper_pin_map.svg

[probe_G38_multi extruder]
# Activate the "probe_G38.py" extras module, providing generalized G38.2 probing.
# The name in the config section **must** match the name of an [extruder] section,
# in order to find the probe object for the current extruder automatically.
#
# Settings borrowed from "[smart_effector]".
recovery_time: 0.0
# Settings from original "[probe]" section.
pin: ^tools:PC5
z_offset: 0


[probe_G38_multi extruder1]
# Activate the "probe_G38.py" extras module, providing generalized G38.2 probing.
# The name in the config section **must** match the name of an [extruder] section,
# in order to find the probe object for the current extruder automatically.
#
# Settings borrowed from "[smart_effector]".
recovery_time: 0.0
# Settings from original "[probe]" section.
pin: ^tools:PB1
z_offset: 0
```