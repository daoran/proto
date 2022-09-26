proto flight-controller firmware
================================

1. Install udev rules for Teensy 4.0 [`udev_rules`_]
2. Install arduino-cli [`install_arduino_cli`_]
3. Configure arduino-cli :code:`arduino-cli config init`
4. Edit the :code:`arduino-cli.yaml` (usually in
   :code:`~/.arduino15/arduino-cli.yaml`) and add
   :code:`https://www.pjrc.com/teensy/td_156/package_teensy_index.json` to the
   list of additional urls in under the board manager section.

.. udev_rules_: https://www.pjrc.com/teensy/00-teensy.rules
.. install_arduino_cli_: https://arduino.github.io/arduino-cli/0.27/installation/
