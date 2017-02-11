#!/usr/bin/env octave -q
#
# Apply fourier transform to pulse-oxy sensor data
#
# Command line:
# octave  precipitation-fourier.m  hly-data-file.csv > /dev/null
#
# Joe Desbonnet,
# jdesbonnet@gmail.com
# 11 Feb 2017

datain = argv(){1}
dataout = argv(){2}

data_matrix = dlmread(datain, " ")

# Extract column #2 (red column) as vector
red_led = data_matrix( : , 2)
ir_led = data_matrix( : , 3)

Ts = 0.028
fs = 1/Ts

#fa = abs(fft(red_led))
fa = abs(fft(ir_led))

N = rows (fa)

n = [1 : 1 : N/2];

# ./ operator is element by element division
period  = N * Ts ./ n ;
freq = n * fs / N;

length (n)
length (freq)
length (fa(1:end/2))

#plot (freq(2:end),fa(2:end/2))

out = horzcat (freq' , fa(1:end/2))
dlmwrite (dataout, out, " " )

#pause()
