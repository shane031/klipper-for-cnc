# Klipper configs for various machines

Mainly for ATmega328p (Arduino+CNC-shield).

The [printer.cfg](./printer.cfg) contains a few lines, each will include a full configuration from the corresponding folder. Uncomment only one of the lines in the file to enable one of the following:

- [printer_minimal.cfg](./printer_minimal.cfg): the bare minimum to run klipper.
- [cnc_shield_xyze](./cnc_shield_xyze): a basic 4-axis / 3D-printer setup.
- [cnc_shield_manual_stepper](./cnc_shield_manual_stepper): 4-axis as manual steppers.
- [pi_pico](./pi_pico): old testing setup with a Raspberry Pi Pico (RP2040).

Browse the README files in those directories for details.
