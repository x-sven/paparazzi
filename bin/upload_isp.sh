#
# $Id:$ olri
#
[ $# = 1 ] || { echo "Usage: $0 <AirframeName>"; exit 5; }

make AIRCRAFT="$1" ap.compile

lpc21isp  var/$AIRCRAFT/ap/ap.hex /dev/ttyS1 38400 12000
