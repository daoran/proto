proto flight-controller firmware
================================

Hardware
--------

- Teensy 4.0
- MPU6050 (6dof IMU sensor)
- BMP280 (temperature and pressure sensor)


Toolchain Setup
---------------

Reference: [pjrc_forum_]

1. Install udev rules for Teensy 4.0 [udev_rules_] to
   :code:`/etc/udev/rules.d/00-teensy.rules`

2. Install arduino-cli [install_arduino_cli_]

3. Configure arduino-cli via :code:`arduino-cli config init`

4. Edit :code:`arduino-cli.yaml` (usually in
   :code:`~/.arduino15/arduino-cli.yaml`) and add
   :code:`https://www.pjrc.com/teensy/td_156/package_teensy_index.json` to the
   list of additional urls under the board manager section. Like so:

.. code-block::

  board_manager:
    additional_urls: [
      https://www.pjrc.com/teensy/td_156/package_teensy_index.json
    ]

5. Install teensy files via :code:`arduino-cli core install teensy:avr`

6. Check arduino-cli can see your teensy board via :code:`arduino-cli board
   list`, you should see something like this:

.. code-block::

  Port         Protocol Type              Board Name FQBN                Core
  /dev/ttyACM0 serial   Serial Port (USB) Unknown
  usb1/1-6     teensy   Teensy Ports      Teensy 4.0 teensy:avr:teensy40 teensy:avr

7. Compile and upload the firmware

.. code-block::

  arduino-cli compile -b teensy:avr:teensy40 <path to arduino sketch>
  arduino-cli upload -b teensy:avr:teensy40 -p usb1/1-6 <path to arduino sketch>


.. _pjrc_forum: https://forum.pjrc.com/threads/53548-Arduino-CLI-Alpha-Release-Teensy-Support?p=299430&viewfull=1#post299430
.. _udev_rules: https://www.pjrc.com/teensy/00-teensy.rules
.. _install_arduino_cli: https://arduino.github.io/arduino-cli/0.27/installation/
