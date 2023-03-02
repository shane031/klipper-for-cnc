Trying to add ABC axes on this fork, by duplicating the trapq object.

What works:

- Movement seems to work :)
- Homing now works.
- Probing with G38 works.

Important TODOs:

- Run tests! Only basic functionality has been covered.
- Limit checks for ABC axes not implemented.
- "Extra" steppers not tested (i.e. `stepper_a1`, etc.)

Perhaps it would have been better/simpler to use multiple extruder axes instead.

----

Welcome to my fork of the Klipper project, with awesome home-able extruders and CNC-style probing!

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

# General docs on Klipper

https://www.klipper3d.org/

Klipper is a 3d-Printer firmware. It combines the power of a general
purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more information on why you should use Klipper.

To begin using Klipper start by [installing](https://www.klipper3d.org/Installation.html) it.

Klipper is Free Software. See the [license](COPYING) or read the [documentation](https://www.klipper3d.org/Overview.html). We depend on
the generous support from our [sponsors](https://www.klipper3d.org/Sponsors.html).

# Fork notes: 7-axis

Config examples at the next section.

This fork implements:

- CNC on XYZABCE axes.
    - Module: core modifications to toolhead, homing, and cartesian kinematics. New "abc" cartesian kinematics: [cartesian_abc.py](./klippy/kinematics/cartesian_abc.py)
    - Commands:
        - Move: `G1 X10 Y10 Z10 A10 B10 C10 E10` (same as a regular G1 command).
        - Home: `G28 A` (same as regular `G28`).
        - Multi-probe: `MULTIPROBE2 PROBE_NAME=myprobe A=-20 B=10 F=2` (same as regular `MULTIPROBE`).
        - And so on ...
    - Caveats: needs testing on longer GCODE programs. Limit checks not implemented. The 3 stepper sections must be configured, _partial_ extra axis is not implemented yet (i.e. XYZ+AB). Extra steppers not tested (i.e. `stepper_a1`).
    - Module incompatibilites: probably many. Tested with `virtual_sdcard`, `pause_resume`, and `force_move`. Alternate kinematics for the XYZ axes not tested.
- Homing on the steppers of `[extruder]`s.
    - Module: [extruder_home.py](./klippy/extras/extruder_home.py)
    - Command: `HOME_ACTIVE_EXTRUDER`.
    - Mux-command: `HOME_EXTRUDER EXTRUDER=extruder` (will activate `extruder` and reactivate the previous extruder when done).
    - Caveats: It is untested on extruder steppers configured as `[extruder_stepper]` later synced to a particular `[extruder]`. No "second home" is performed.
- Probing in arbitrary directions with `G38.2`, `G38.3`, `G38.4`, and `G38.5` (single-probe version).
    - Module: [probe_G38.py](./klippy/extras/probe_G38.py)
    - Example command: `G38.2 X20 F10`
    - For reference, see [LinuxCNC](http://linuxcnc.org/docs/stable/html/gcode/g-code.html#gcode:g38)'s definition of _G38.n Straight Probe_ commands.
    - Note: affected by `G90`/`G91` and `M82`/ `M83`.
    - Module incompatibilites: `[probe_G38_multi ...]`
- General probing with multiple probe pins is supported by an experimental module:
    - Module: [probe_G38_multi.py](./klippy/extras/probe_G38_multi.py)
    - Example multi-command: `MULTIPROBE2 PROBE_NAME=extruder1 Z=-20 F=1` (replace the `2` in `MULTIPROBE2` with `3`, `4`, or `5` for the other probing modes).
    - Example mono-command: `G38.2 X20 F10` (replace `.2` by `.3-.5` for the other probing modes). To choose the probe pin, this command will try to match the probe's config name to an extruder name, or fail.
    - The probes can be queried with `QUERY_ENDSTOPS` (instead of `QUERY_PROBE`).
    - Note: affected by `G90`/`G91` and `M82`/ `M83`.
    - Known incompatibilites: `[probe_G38]`
- The `SET_KINEMATIC_POSITION` command now works with extruder position as well.
    - Try this out: `SET_KINEMATIC_POSITION E=66.6`
- Absolute extruder moves are now absolute.
    - You can now count on absolute coordinate systems staying that way unless you update them explicitly (e.g. with `G92 E0` and similar commands).
    - The origin used to be altered without warning after every tool-change (i.e. extruder activation) in a way equivalent to sending `G92 E0`. This means that the extruder's origins were effectively relative to the last position of the extruder before a toolchange, which was enforced in Klipper to support the obscure expectations of old slicers.
    - See discussion at: https://klipper.discourse.group/t/6558

Rather minor modifications in Klippy's core were made to accommodate these features. This fork exists because a massive amount of effort is needed to merge new stuff into "main" Klipper. Fortunately this means that pull requests would be very welcome here, even for cool but half-baked features.

## Interested in CNC stuff for Klipper?

Let's chat over here: https://klipper.discourse.group/t/klipper-forks-for-cnc/5698

Cheers!

## Configs

See examples here: [config-pi-pico-mainsail](./config/configs-pipetting-bot/config-pi-pico-mainsail)

These are meant as reference configs; you _must_ adjust them to match your setup first.

### Extra ABC axes config

See examples here: [labo-robot-pinmap](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap)

Configure extra ABC kinematics: [printer.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap/printer.cfg)

```yaml
[printer]
# Regular pritner configuration.
kinematics: cartesian
# Units: mm/s and mm/s^2:
max_velocity: 5000    # F120000
max_z_velocity: 250   # F30000
max_accel: 1000
# Add ABC kinematics to the toolhead:
kinematics_abc: cartesian_abc
axis: XYZABC
```

Configure the additional ABC steppers: [printer_steppers_abc.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap/printer_steppers_abc.cfg)

```yaml
[stepper_a]
# Regular stepper configuration.
# ...

[stepper_b]
# Regular stepper configuration.
# ...

[stepper_c]
# Regular stepper configuration.
# ...
```

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
homing_positive_dir: False  # ADJUST TO MATCH YOUR SETUP
endstop_pin: gpio18  # REPLACE WITH THE PIN OF **YOUR** HOMING ENDSTOP

[extruder_home extruder1]
# No parameters needed.
```

Note that the `[extruder]` must have an "endstop_pin" defined for it to be home-able. It is otherwise setup as a "regular" extruder, and a corresponding `[extruder_home extruder]` section will not work as exected. For instance, a `HOME_EXTRUDER EXTRUDER=extruder` command fail with this error: `'MCU_stepper' object has no attribute 'get_endstops'`

### Single-probe config

Config: [probe_G38.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap/probe_G38.cfg)

```yaml
[probe_G38]
recovery_time: 0.4
pin: gpio19
z_offset: 0
```

Note that `[probe_G38]` is incompatible with `[probe_G38_multi extruder]`.

### Multi-probing config

Note that this module also implements the regular `G38.n` commands, using the probe section associated to the active extruder's name. 

Config: [probe_G38_multi.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap/probe_G38_multi.cfg)

```yaml
[probe_G38_multi extruder]
recovery_time: 0.0
pin: ^tools:PC5
z_offset: 0


[probe_G38_multi my_probe]
recovery_time: 0.0
pin: ^tools:PB1
z_offset: 0
```

For convenience, their status can show up next to the endstops in Mainsail:

![query_probe_endstops.png](./docs/img/pipetting/query_probe_endstops.png)

# Use cases

- More general CNC usage.
- Syringe extruders.
- Pipetting / liquid-handling robots.

# Installation

The easiest way is to use a KIAUH "klipper_repos.txt" file. Details at: https://github.com/th33xitus/kiauh/blob/master/klipper_repos.txt.example

1. SSH into the Pi.
2. Copy "klipper_repos.txt.example" to "klipper_repos.txt".
    - Use the command: `cp kiauh/klipper_repos.txt.example  kiauh/klipper_repos.txt`
4. Edit the `kiauh/klipper_repos.txt` file to append "`naikymen/klipper-homing-extruder,pipetting`" after the last line.
    - Use the command: `echo "naikymen/klipper-homing-extruder,pipetting" >> kiauh/klipper_repos.txt`
5. Start KIAUH.
    - Use the command: `./kiauh/kiauh.sh`
7. Choose option "`6) [Settings]`".
8. Choose option "`1) Set custom Klipper repository`".
9. Choose the option corresonding to "`naikymen/klipper-homing-extruder -> pipetting`"
10. Use KIAUH to uninstall and reinstall Klipper.

## Updates through moonraker

### Using the fork

I've made a Moonraker fork to avoid the issues explained below: https://github.com/naikymen/moonraker-pipetting

Keeing it updated will be cumbersome, but there is no alternative to get everything tidy:

![moonraker_fork.png](./docs/img/pipetting/moonraker_fork.png)

To install, use SSH to replace the moonraker directory. Go to mainsail, restart moonraker, and refresh the "Update Manager". If an "invalid" state appears, click the button and do a "hard reset". Everything should look nice!

### Editing the source

Unfortunately the differing repo and branch names dont play well with the hard-coded stuff in Moonraker.

However by applying this simple diff to `~/moonraker` it will just work:

```diff
diff --git a/moonraker/components/simplyprint.py b/moonraker/components/simplyprint.py
index a3bfac3..ba33057 100644
--- a/moonraker/components/simplyprint.py
+++ b/moonraker/components/simplyprint.py
@@ -1002,7 +1002,8 @@ class SimplyPrint(Subscribable):
             "firmware": "Klipper",
             "firmware_version": version,
             "firmware_date": firmware_date,
-            "firmware_link": "https://github.com/Klipper3d/klipper",
+            # "firmware_link": "https://github.com/Klipper3d/klipper",
+            "firmware_link": "https://github.com/naikymen/klipper-homing-extruder",
         }
         diff = self._get_object_diff(fw_info, self.cache.firmware_info)
         if diff:
diff --git a/moonraker/components/update_manager/base_config.py b/moonraker/components/update_manager/base_config.py
index 4bcccbc..7c48aef 100644
--- a/moonraker/components/update_manager/base_config.py
+++ b/moonraker/components/update_manager/base_config.py
@@ -34,7 +34,9 @@ BASE_CONFIG: Dict[str, Dict[str, str]] = {
     },
     "klipper": {
         "moved_origin": "https://github.com/kevinoconnor/klipper.git",
-        "origin": "https://github.com/Klipper3d/klipper.git",
+        # "origin": "https://github.com/Klipper3d/klipper.git",
+        "origin": "https://github.com/naikymen/klipper-homing-extruder.git",
+        "primary_branch": "pipetting",
         "requirements": "scripts/klippy-requirements.txt",
         "venv_args": "-p python2",
         "install_script": "scripts/install-octopi.sh",
```

![pipetting_moonraker.png](./docs/img/pipetting_moonraker.png)

Since this modifies the Moonraker source, now Moonraker won't be updatable because it will have diverged and thus "invalid" or "dirty" (rawr?).

![moonraker_pipetting.png](./docs/img/moonraker_pipetting.png)

A price I am willing to pay (?)

Raise your hand here to change this: https://github.com/Arksine/moonraker/issues/615

