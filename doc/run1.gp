set terminal pngcairo size 1024,600
set output 'run1.png'
set title 'MAX30102 test run
set xlabel 'Time (seconds)'
set ylabel 'LED current ADC units (62.5pA per unit)' 
plot \
'run1.dat' using (($1 - 1445214196)/30e6):($2-1.99e6) with lines lw 2 title 'RED', \
'' using (($1 - 1445214196)/30e6):($3 - 1.115e6 - 1.99e6) with lines linecolor "#8000FF" lw 2 title 'IR'


