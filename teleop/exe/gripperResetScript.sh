#!/bin/bash

#Right gripper
while true
do
    sudo sns -a pciod-rgripper; result=$?
    if [ $result -ne 0 ] 
        then
        #echo "Restarting pciod-rgripper deamon..."
        sudo pciod -d -I pciod-rgripper -c 'rgripper-cmd' -s 'rgripper-state' -b 10 -m 12
    fi

    sudo sns -a pciod-lgripper; result=$?
    if [ $result -ne 0 ] 
        then
        #echo "Restarting pciod-lgripper deamon..."
        sudo pciod -d -I pciod-lgripper -c 'lgripper-cmd' -s 'lgripper-state' -b 2 -m 12
    fi

    sleep 0.1 # 100 msec 

done
