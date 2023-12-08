#!/bin/bash

cd ~/teensy_ws/agrobot
pio run --target clean_microros
pio lib install
pio run