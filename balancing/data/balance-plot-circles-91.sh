#!/bin/bash
input=$( printf 'balance-dump-%05d.dat' $1 )
output1=$( printf 'plots/balance-plot-%05d-a.png' $1 )
output2=$( printf 'plots/balance-plot-%05d-b.png' $1 )
output3=$( printf 'plots/balance-plot-%05d-c.png' $1 )
output4=$( printf 'plots/balance-plot-%05d-d.png' $1 )

if [ "$2" != "--display" ]
then
	gnuplot -e "filename='$input';out1='$output1';out2='$output2';out3='$output3';out4='$output4';xstart=5" balance-spin-xy-plots.plg
fi

eog $output1 &
eog $output2 &
eog $output3 &
eog $output4 &
