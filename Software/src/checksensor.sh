#!/bin/sh

	echo 17 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio17/direction
	echo 0 > /sys/class/gpio/gpio17/value

	echo $LANG
	export LANG=ja_JP.UTF-8
	echo $LANG

	cd /home/pi/
	python /home/pi/tsubuta.py


	echo 17 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio17/direction
	echo 1 > /sys/class/gpio/gpio17/value

	sleep 10


