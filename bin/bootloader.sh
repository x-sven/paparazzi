#!/bin/sh
#
# $Id$ olri
#

make upload_bl PROC=GENERIC

for TTY in /dev/ttyS0 /dev/ttyS1 /dev/ttyUSB0 /dev/ttyUSB1
do
    if [ -c $TTY ] ; then
	lpc21isp sw/airborne/arm7/test/bootloader/bl.hex $TTY 38400 12000
    fi
done
