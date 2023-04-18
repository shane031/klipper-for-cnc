#!/usr/bin/python
# The two commands needed to run a "batch" test.
# Note that this does not output MCU commands for a manual stepper move, unfortunately.
# An alternative is to "log all incoming queue_step commands" in the "add_move" function at "stepcompress.c". Uncomment the "errorf" calls there.
python ./klippy/klippy.py ../printer_data/config/cnc_shield_xyze/printer.cfg -i test.gcode -o test.serial -v -d out/klipper.dict
python ./klippy/parsedump.py out/klipper.dict test.serial > test.txt
