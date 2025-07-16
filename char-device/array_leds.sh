#!/bin/bash
#
# Test file to blink led using /dev/simplegpio/ file entry

on_time=${1:-0.5}
off_time=${1:-0.5}
running=true

if [ -e "/dev/simplegpio" ]; then

	cleanup(){
		running=false
		echo -n '0' > /dev/simplegpio
	}
	
	trap cleanup SIGINT SIGTERM
	
	value=0
	while $running; 
	do
		echo -n $value > /dev/simplegpio
		sleep $on_time
		value=$((value+1))
	done
else
	echo "/dev/simplegpio device entry is not there, load the respective module"
fi
