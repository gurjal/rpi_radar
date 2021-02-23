#!/bin/sh

echo "2" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio2/direction
#echo "1" > /sys/class/gpio/gpio2/value

echo "3" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio3/direction
#echo "1" > /sys/class/gpio/gpio3/value

echo "4" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio4/direction
#echo "1" > /sys/class/gpio/gpio4/value

#echo "14" > /sys/class/gpio/export
#echo "out" > /sys/class/gpio/gpio14/direction
##echo "1" > /sys/class/gpio/gpio14/value
#
#echo "15" > /sys/class/gpio/export
#echo "out" > /sys/class/gpio/gpio15/direction
##echo "1" > /sys/class/gpio/gpio15/value
#
#echo "27" > /sys/class/gpio/export
#echo "out" > /sys/class/gpio/gpio27/direction
##echo "1" > /sys/class/gpio/gpio27/value

