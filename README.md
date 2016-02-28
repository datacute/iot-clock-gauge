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

Unfortunately the project wasn't one of the Phase 1 winners.

With no MKR1000, this project had to switch to targetting another
platform, using devices Stephen already has (Arduino UNO, DS1307
RTC, Wemos D1 mini, Sparkfun ESP8266 Thing).

The UNO, DS1307 RTC and LEDs were assembled into a working clock within
a couple of evenings, but the inability to configure or extend it needed
to be solved.

Three options were considered:

1. Use an ESP8266 as a wifi shield for the UNO
  - Need a level shifter to communcate with 3.3V ESP8266
  - Takes up more space
  - UNO will probably run out of RAM
2. Using a UNO to control the clock and act as an I2C slave, with the
   ESP8266 as a web & MQTT configurable controller.
  - Need a level shifter to communcate via I2C with 5V UNO and RTC
  - Takes up more space
  - More complicated communications
3. Doing everything on an ESP8266.
  - No longer meets the requirements of the Arduino Maker Challenge
  - Need a level shifter to communicate with the LEDs
  - Need another different level shifter to communcate with 5V I2C RTC
    - Or a 3.3V RTC
    - Or (last resort/temporary measure) use ntp instead of an RTC

The decision was made to go with option 3.
- 3.3V RTC ordered and received (DS3231 board)
- LED signal level shifter ordered again (SN74AHCT125N)

Until the level shifters arrive, the LEDs are ticking away at a reduced
brightness, due to being powered at 3.3V
