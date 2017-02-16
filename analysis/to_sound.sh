#!/bin/bash
# Convert MAX30102 sensor output to wav file
cut -f 5 -d ' ' raw.dat | sox -t raw -r 800 -e signed-integer -b 16  - t.wav
