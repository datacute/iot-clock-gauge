# IoT Wall Gauge and Clock
By Stephen Denne

With an MKR1000 attached to a ring of bright RGB LEDs, a colourful wall
clock can be created.

By receiving messages from Azure IoT Hub,the clock becomes highly
configurable, and can also double as an information radiator, displaying
your data in a circular gauge on your wall.

I plan on creating a Windows Universal Application to configure the IoT
Wall Gauge and Clock:

- Time zone
- Colour schemes used by the clock
- Creating count-down timers and visual alarms
- Sunrise/Sunset/Position of Sun & Moon visualisation (clockwise or
  anti-clockwise to suit both northern and southern hemispheres)
- IoT Hub messages to receive, and how to convert those messages to a
  circular display on the gauge
- Cycling/Mixing/Fading between multiple visualisations

The Windows Universal Application will also provide a preview of the
display, for the configuration being edited, before using Azure IoT Hub or
Windows Remote Arduino to send the configuration to the IoT Wall Gauge and
Clock.

The the time of the day, and the ambient light level, will be configurable
to be either locally attached hardware or obtained from other devices via
the Azure IoT Hub. Attaching these two additional hardware components will
enable the clock to function meaningfully, and adjust its brightness to the
room, even without an internet connection.

---------------------------------------------------------------------------

The above [project idea](https://www.hackster.io/challenges/arduino-microsoft-maker/ideas/1045)
was submitted to the "[World’s Largest Arduino Maker Challenge](https://www.hackster.io/challenges/arduino-microsoft-maker)"
on [hackster.io](https://www.hackster.io/)
