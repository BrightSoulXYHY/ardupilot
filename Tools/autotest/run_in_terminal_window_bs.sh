#!/bin/bash

# Try to run a command in an appropriate type of terminal window
# depending on whats available
# Sigh: theres no common way of handling command line args :-(
name="$1"
shift
echo "RiTW: Starting $name : $*"


filename="/tmp/$name.log"
echo "RiTW: WSL not using Window access, logging to $filename"
cmd="$1"
shift
# the following "true" is to avoid bash optimising the following call
# to avoid creating a subshell.  We need that subshell, or
# _fdm_input_step sees ArduPilot has no parent and kills ArduPilot!
( : ; "$cmd" $* &>"$filename" < /dev/null ) &



red=`tput setaf 5`
reset=`tput sgr0`
cat $filename | while read line
do
    echo $red $line $reset
done


exit 0
