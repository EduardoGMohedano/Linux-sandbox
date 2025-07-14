#!/bin/bash
#
# Test file to blink led using /proc/lll-gpio entry


if [ -f "/proc/lll-gpio" ]; then
	while : 
	do
		echo -n '0' > /proc/lll-gpio
		sleep 0.3
		echo -n '1' > /proc/lll-gpio
		sleep 0.3
	done
else
	echo "Proc entry is not there, load the respective module"
fi
