# Control Software for Chimera
This is the software compatible with Anaero Technology's Chimera gas sensor.
The software here is designed to run on the ESP32s2 within the chimera itself and allows for communication with a computer using a USB-C cable and the software found here: https://github.com/Anaero-Technology/Chimera-Python-Controller

## How to use this
In order to install the software you will need the Arduino IDE which can be found here: https://www.arduino.cc/en/software
### Setup
Under 'Tools', 'Board' and 'Board Manager' make sure you have 'Arduino ESP32 Boards' installed.
Under 'Sketch', 'Include Library' and 'Manage Libraries' make sure you have 'ESP32AnalogRead' installed.
### Uploading
Connect the Chimera to your computer using a USB-C cable.
On the board there are two buttons, one marked 'boot' and the other 'reset'. Press and hold boot, then press and release reset followed by releasing boot.
You should then see under 'Tools' and 'Port' that a port has become available, select it. If there is not a port check your cable is working and try connecting again.
Under 'Tools' and 'Board' select 'ESP32s2 Dev Module' which should be under 'ESP32 Arduino'.
Finally under 'Tools' make sure that 'USB CDC on Boot' is enabled.
Then press the upload button (or ctrl+u) and wait for the code to be transferred. It is normal to receive an error at the end which says 'no_reset'.

Then unplug the device fully to allow it to restart, then attempt to connect with the Python interface.
