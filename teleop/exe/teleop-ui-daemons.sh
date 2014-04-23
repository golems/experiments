#!/bin/bash

DO_LIBERTY=

################################################################################
## ARGUMENT HANDLING
################################################################################
while getopts "l" OPTION
do
    case $OPTION in
        l)
            DO_LIBERTY="yes"
            ;;
    esac
done

#### start up daemons
ach mk spacenav-data-l -o 666 -1
ach mk spacenav-data-r -o 666 -1
nohup jachd -j 1 -c spacenav-data-l -I jachd-spacenav >/dev/null &
nohup jachd -j 0 -c spacenav-data-r -I jachd-spacenav2 >/dev/null &

#### make vis channel b/c saul's code expects it:
ach mk telop-05-workspce-vis -o 666 -1

if [[ -n $DO_LIBERTY ]] 
then
    echo "pulling liberty channel from hubo-remote.local"
    ach mk liberty -o 666 -1
    achd pull -d -r -v hubo-remote.local liberty
fi

#### wait until user is done doing whatever it is they're doing
echo "Daemons started, press enter to clean up"
read foo

#### and kill our daemons
sns -k jachd-spacenav 
sns -k jachd-spacenav2

if [[ -n $DO_LIBERTY ]] 
then
    echo "cleaning up hubo-remote.local"
    pgrep -f "achd pull -d -r -v hubo-remote.local liberty" | xargs -I pids kill -9 pids
    ach rm liberty
fi
