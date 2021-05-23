set -e
# make release

ARDUINO=~/Downloads/arduino-1.8.13/arduino
$ARDUINO --upload firmware/firmware.ino --port /dev/ttyACM0
# $ARDUINO --upload firmware/firmware.ino
