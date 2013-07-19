#!/bin/bash




#### start up daemons
nohup jachd -j 2 -c spacenav-data -I jachd-spacenav >/dev/null &
nohup jachd -j 1 -c spacenav2-data -I jachd-spacenav2 >/dev/null &
ach mk liberty -o 666 -1
achd push -d -r -v hubo-remote.local liberty

#### wait until user is done doing whatever it is they're doing
echo "Daemons started, press enter to clean up"
read foo

#### and kill our daemons
sns -k jachd-spacenav 
sns -k jachd-spacenav2
pgrep -f "achd push -d -r -v hubo-remote.local liberty" | xargs -I pids kill -9 pids
ach rm liberty
