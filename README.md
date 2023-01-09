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
    - See: [extruder_home.py](./klippy/extras/extruder_home.py)
    - Note: this may not work on extruder steppers configured as `[extruder_stepper]` later synced to an `[extruder]`.
- Probing in arbitrary directions with `G38.2`, `G38.3`, `G38.4`, and `G38.5`.
    - See: [probe_G38.py](./klippy/extras/probe_G38.py)
    - For reference, see [LinuxCNC](http://linuxcnc.org/docs/stable/html/gcode/g-code.html#gcode:g38)'s definition of _G38.n Straight Probe_ commands.

Minor modifications in Klippy's core were made to accommodate these features.

## Configs

See examples here: [config-pi-pico-mainsail](./config/configs-pipetting-bot/config-pi-pico-mainsail)

### Minimal homing config

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

```yaml
[probe_G38]
recovery_time: 0.4
pin: gpio19
z_offset: 0
```

# Interested in CNC stuff for Klipper?

Let's chat over here: https://klipper.discourse.group/t/klipper-forks-for-cnc/5698

Cheers!