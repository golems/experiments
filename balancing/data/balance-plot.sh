#!/bin/bash
input=$( printf 'balance-dump-%05d.dat' $1 )
output1=$( printf 'plots/balance-plot-%05d-a.png' $1 )
output2=$( printf 'plots/balance-plot-%05d-b.png' $1 )
output3=$( printf 'plots/balance-plot-%05d-c.png' $1 )

gnuplot -e "filename='$input';out1='$output1';out2='$output2';out3='$output3';xstart=5" balance-plots.plg

display $output1 &
display $output2 &
display $output3
