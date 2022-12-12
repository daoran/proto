#!/bin/bash
set -e

# arduino-cli compile -b teensy:avr:teensy40 firmware
# arduino-cli upload -b teensy:avr:teensy40 -p usb1/1-6 firmware
# python3 firmware/firmware_debugger.py

tmux send-keys -t dev -R C-l C-m
tmux send-keys -t dev -R "\
  cd ~/projects/proto/proto_mav \
  && arduino-cli compile -b teensy:avr:teensy40 firmware \
  && arduino-cli upload -b teensy:avr:teensy40 -p usb1/1-6 firmware
" C-m C-m
exit

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cu -l /dev/ttyACM0 -s 115200
# " C-m C-m
# exit
