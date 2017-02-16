#!/bin/bash
# Convert MAX30102 sensor output to wav file.
# Sensor output piped into stdin.
cut -f 5 -d ' ' - | sox -t raw -r 1200 -e signed-integer -b 16  - t.wav
