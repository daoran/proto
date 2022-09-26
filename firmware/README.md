# proto flight-controller firmware

1. Install udev rules for Teensy 4.0 [udev_rules_][1] to
   `/etc/udev/rules.d/00-teensy.rules`

2. Install arduino-cli [install_arduino_cli_][2]

3. Configure arduino-cli via `arduino-cli config init`

4. Edit `arduino-cli.yaml` (usually in
   `~/.arduino15/arduino-cli.yaml`) and add
   `https://www.pjrc.com/teensy/td_156/package_teensy_index.json` to the
   list of additional urls under the board manager section. Like so:

```
  board_manager:
    additional_urls: [
      https://www.pjrc.com/teensy/td_156/package_teensy_index.json
    ]
```

5. Install teensy files via `arduino-cli core install teensy:avr`

6. Check arduino-cli can see your teensy board via `arduino-cli board
   list`, you should see something like this:

```
  Port         Protocol Type              Board Name FQBN                Core
  /dev/ttyACM0 serial   Serial Port (USB) Unknown
  usb1/1-6     teensy   Teensy Ports      Teensy 4.0 teensy:avr:teensy40 teensy:avr
```

7. Compile and upload the firmware

```
  arduino-cli compile -b teensy:avr:teensy40
  arduino-cli upload -b teensy:avr:teensy40 -p usb1/1-6
```

$$ x = {-b \pm \sqrt{b^2-4ac} \over 2a} $$

[1]: _udev_rules: https://www.pjrc.com/teensy/00-teensy.rules
[2]: _install_arduino_cli: https://arduino.github.io/arduino-cli/0.27/installation/
