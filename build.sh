#!/bin/bash

cd ~/teensy_ws/agrobot
pio run

cd .pio/build/teensy41
teensy_loader_cli --mcu=TEENSY41 -sv firmware.hex