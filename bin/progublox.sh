#/bin/sh
#
# $Id:$ olri
#
(
    cd $PAPARAZZI_HOME/conf/gps
    rm -f ublox_conf
#    make MODCFLAGS='-DDEFAULT_BAUDRATE=9600 -DOUT_FILE_NAME=\"/dev/ttyS1\"' && ./ublox_conf
    rm -f ublox_conf
    make MODCFLAGS='-DDEFAULT_BAUDRATE=38400 -DOUT_FILE_NAME=\"/dev/ttyS1\"' && ./ublox_conf
)
