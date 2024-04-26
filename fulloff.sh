#!/bin/sh

if [ ! -d /sys/class/gpio/gpio24 ]
then
    echo "24" > /sys/class/gpio/export
fi

if [ ! -d /sys/class/gpio/gpio4 ]
then
    echo "4" > /sys/class/gpio/export
fi

sleep 0.1
echo "out" > /sys/class/gpio/gpio24/direction
echo 0 > /sys/class/gpio/gpio24/value
echo "out" > /sys/class/gpio/gpio4/direction
echo 0 > /sys/class/gpio/gpio4/value
