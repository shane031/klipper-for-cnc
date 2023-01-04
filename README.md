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

- Homing on the extruder steppers.
    - See: [extruder_home.py](./klippy/extras/extruder_home.py)
- Probing in arbitrary directions with `G38.2` and `G38.3`.
    - See: [probe_G38.py](./klippy/extras/probe_G38.py)
    - `G38.4` and `G38.5` could work by inverting the endstop pin signal, but implementing all four commands seems harder.
    - If someone knows how to have klipper trigger and endstop on deactivation and activation, all four commands could be implemented.

Minor modifications in Klippy's core were made to accommodate these features.

# Interested in CNC stuff for Klipper?

Let's chat over here: https://klipper.discourse.group/t/klipper-forks-for-cnc/5698

Cheers!