#include <Wire.h>
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC
#include <Time.h>         //https://github.com/PaulStoffregen/Time
#include <TimeLib.h>      //https://github.com/PaulStoffregen/Time

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <WiFiUdp.h>
#include <PubSubClient.h>

const char* ssid = "your network SSID (name)";  //  your network SSID (name)
const char* password = "your network password";       // your network password

const char* logTopic = "home/clock/log";
const char* subscribeTopic = "home/clock/incoming"; // subscribe to this topic; anything sent here will be passed into the messageReceived function
IPAddress mqtt_server(192,168,0,1); // your MQTT Server
String mqttClientName = "clock-"; // just a name used to talk to MQTT broker
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

long lastMsg = 0;
char msg[50];
int value = 0;

// NTP Servers:
IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 102); // time-b.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 103); // time-c.timefreq.bldrdoc.gov

const int timeZone = 13;     // NZDT
//const int timeZone = 1;     // Central European Time
//const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)


WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
time_t getNtpTime();
void digitalClockDisplay();
uint8_t mod60(int8_t v);
void sendNTPpacket(IPAddress &address);

#define PIN_BRIGHTNESS_BUTTON D8
#define PIN_NEOPIXEL_RING D5

enum PaletteColour { QUARTER_TICK, FIVE_MINUTE_TICK, HOUR, HOUR1, HOUR2, MINUTE, MINUTE1, SECOND};

uint32_t palette0[] = {0xFFFFFF,0x404040,0xFF0000,0x400000,0x200000,0x00FF00,0x004000,0x0000FF};

uint32_t* palettes[]={
  &palette0[0]
};

int currentPalette = 0;

int brightness = 16;

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN_NEOPIXEL_RING, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

time_t prevDisplay = 0; // when the digital clock was displayed
unsigned long timemslast;
int speedy = 0;
bool enableSpeedy = false;
bool displaySpeedy = false;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else if ((char)payload[0] == '0') {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  } else if ((char)payload[0] == 'B') {
    brightness = 1<<(payload[1]-'0');
    strip.setBrightness(brightness);
    digitalClockDisplay();
    mqttClient.publish(logTopic, "Brightness Set");
  } else if ((char)payload[0] == 'N') {
    mqttClient.publish(logTopic, "Synchronizing");
    time_t t = getNtpTime();
    RTC.set(t);
    setTime(t);
    mqttClient.publish(logTopic, "Synchronized");
  } else if ((char)payload[0] == 'S') {
    if (enableSpeedy)
    {
      enableSpeedy = false;
      mqttClient.publish(logTopic, "Second sweep disabled");
    } else {
      enableSpeedy = true;
      mqttClient.publish(logTopic, "Second sweep enabled");
    }
  }
}

// Wifi state management
enum wifiState {
  wifiOff, wifiBegun, wifiConnected
};
wifiState wifiCurrentState = wifiOff;

void wifiBegin() {
  wifiCurrentState = wifiBegun;
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
}

void wifiWaitForConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiCurrentState = wifiConnected;
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Get ready for NTP
    Udp.begin(localPort);
    //time_t t = getNtpTime();
    //RTC.set(t);
    //setTime(t);
  
    // Get ready for MQTT
    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setCallback(callback);
  }
}

void wifiMaintainConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiBegin();
  }
}

void wifiStateManagement() {
  switch (wifiCurrentState) {
    case wifiOff: wifiBegin(); break;
    case wifiBegun: wifiWaitForConnection(); break;
    default: wifiMaintainConnection();
  }
}

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

int mqttState = 0;

void mqttConnect() {
  Serial.print("Attempting MQTT connection to ");
  Serial.print(mqtt_server);
  Serial.print(" as ");
  Serial.println(mqttClientName);
  // this is still blocking, when a solution is found, mqttState of 1 will mean connecting.
  if (mqttClient.connect((char*) mqttClientName.c_str())) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    mqttClient.publish(logTopic, mqttClientName.c_str());
    // ... and resubscribe
    mqttClient.subscribe(subscribeTopic);
    Serial.print("Subscribed to: ");
    Serial.println(subscribeTopic);
    mqttState = 2;
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" reset to try again");
    mqttState = 3;
  }
}

void mqttStateManagement() {
  if (mqttState == 0) {
    // Generate client name based on MAC address and last 8 bits of microsecond counter
    uint8_t mac[6];
    WiFi.macAddress(mac);
    mqttClientName += macToStr(mac);
    mqttClientName += "-";
    mqttClientName += String(micros() & 0xff, 16);
    mqttConnect();
  } else if (mqttClient.connected()) {
    mqttState = 2;
  } else {
    // mqtt connection attempts are blocking, so if it fails, don't try again.
    // reset to try again.
  }
}

void runClockLoop() {
  if (timeStatus() != timeNotSet) {
    bool displayClock=false;
    unsigned long timemsnow = millis();
    time_t currentTime = now();

    if (timemslast < timemsnow - 15) {
      timemslast = timemsnow;
      speedy=mod60(speedy+1);
      displayClock=true;
    }

    if (currentTime != prevDisplay) {
      displayClock=true;
      timemslast = timemsnow;
      speedy=0;
      displaySpeedy = prevDisplay > 0;
      prevDisplay = currentTime;
    }

    if (displayClock) {
      digitalClockDisplay();
    }
  }
}

void runMqttLoop() {
  if (wifiCurrentState == wifiConnected) {
    mqttStateManagement();
    if (mqttState == 2) {
      mqttClient.loop();
    }
  }
}

int lastButtonState = 0;
int brightnessIncreasing = 0;

void toggleBrightnessWithButton() {
  int buttonState = digitalRead(PIN_BRIGHTNESS_BUTTON);
  if (buttonState != lastButtonState)
  {
    lastButtonState = buttonState;
    if (buttonState == 1)
    {
      if (brightnessIncreasing)
      {
        if (brightness == 0) {
          brightness = 1;
        } else if (brightness == 128) {
          brightness = 255;
        } else if (brightness == 255) {
          brightness = 128;
          brightnessIncreasing = 0;
        } else {
          brightness = brightness << 1;
        }
      } else {
        brightness = brightness >> 1;
        if (brightness == 0) {
          brightnessIncreasing = 1;
        }
      }
      Serial.println(brightness);
      strip.setBrightness(brightness);
      digitalClockDisplay();
    }
  }  
}

void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code

  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH

  pinMode(PIN_BRIGHTNESS_BUTTON, INPUT);

  Serial.begin(115200);

  Serial.println("Setting Up RTC");
  setSyncProvider(RTC.get);
  if(timeStatus() != timeSet) 
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");      

  Serial.println("Turning on Clock LEDs");
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(brightness);
  digitalClockDisplay();
}

void loop() {
  wifiStateManagement();
  runClockLoop();
  runMqttLoop();
  toggleBrightnessWithButton();
}

void digitalClockDisplay(){
  int hours = hour();
  while (hours >= 12) hours -= 12;
  int minutes = minute();
  int seconds = second();
  int hoursPixel = hours * 5 + minutes/12;
  
  uint32_t* p = palettes[currentPalette];
  
  strip.clear();
  strip.setPixelColor(mod60(hoursPixel-2), p[HOUR2]); // hours in blue
  strip.setPixelColor(mod60(hoursPixel-1), p[HOUR1]); // hours in blue
  strip.setPixelColor(mod60(hoursPixel+1), p[HOUR1]); // hours in blue
  strip.setPixelColor(mod60(hoursPixel+2), p[HOUR2]); // hours in blue
  strip.setPixelColor(mod60(minutes-1), p[MINUTE1]); // minutes in green
  strip.setPixelColor(mod60(minutes+1), p[MINUTE1]); // minutes in green
  strip.setPixelColor(0, p[QUARTER_TICK]);
  strip.setPixelColor(5, p[FIVE_MINUTE_TICK]);
  strip.setPixelColor(10, p[FIVE_MINUTE_TICK]);
  strip.setPixelColor(15, p[QUARTER_TICK]);
  strip.setPixelColor(20, p[FIVE_MINUTE_TICK]);
  strip.setPixelColor(25, p[FIVE_MINUTE_TICK]);
  strip.setPixelColor(30, p[QUARTER_TICK]);
  strip.setPixelColor(35, p[FIVE_MINUTE_TICK]);
  strip.setPixelColor(40, p[FIVE_MINUTE_TICK]);
  strip.setPixelColor(45, p[QUARTER_TICK]);
  strip.setPixelColor(50, p[FIVE_MINUTE_TICK]);
  strip.setPixelColor(55, p[FIVE_MINUTE_TICK]);
  strip.setPixelColor(hoursPixel, p[HOUR]);
  strip.setPixelColor(minutes, p[MINUTE]);
  strip.setPixelColor(seconds, p[SECOND]);
  if (displaySpeedy && enableSpeedy) strip.setPixelColor(speedy, 0xFF8800);
  strip.show();
}

uint8_t mod60(int8_t v)
{
  while (v < 0) v += 60;
  while (v >= 60) v -= 60;
  return v;
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  //Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      //Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  //Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

