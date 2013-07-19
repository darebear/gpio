#!/bin/sh

if [ $1 = 1 ]
then
	#echo "Turning GPIO on.."
        echo $2 > /sys/class/gpio/export
        echo out > /sys/class/gpio/gpio$2/direction
        echo $2 > /sys/class/gpio/gpio$2/value
else 
	#echo "Turn GPIO off :("
        echo $2 > /sys/class/gpio/export
        echo out > /sys/class/gpio/gpio$2/direction
        echo 0 > /sys/class/gpio/gpio$2/value
fi
	
#echo "GPIO $2 value is: "
#cat /sys/class/gpio/gpio$2/value

