#include <FS.h>                   //this needs to be first, or it all crashes and burns...

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

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

//flag for saving data
bool shouldSaveConfig = false;

char logTopic[60] = "home/clock/log";
char commandTopic[60] = "home/clock/command"; // subscribe to this topic; anything sent here will be passed into the messageReceived function
char mqttServer[40] = "";
char mqttPort[6] = "1883";
String mqttClientName = "clock-"; // just a name used to talk to MQTT broker
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

long lastMsg = 0;
char msg[50];
int value = 0;

// NTP Servers:
char ntpServer[40] = "pool.ntp.org";

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
void sendNTPpacket(const char *address);

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

WiFiManager wifiManager;
WiFiManagerParameter *custom_mqtt_server;
WiFiManagerParameter *custom_mqtt_port;
WiFiManagerParameter *custom_log_topic;
WiFiManagerParameter *custom_command_topic;
WiFiManagerParameter *custom_ntp_server;

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

//callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

//callback notifying us of the need to save config
void saveConfig() {
  Serial.println("Saving config");

  //read updated parameters
  strcpy(mqttServer, custom_mqtt_server->getValue());
  strcpy(mqttPort, custom_mqtt_port->getValue());
  strcpy(logTopic, custom_log_topic->getValue());
  strcpy(commandTopic, custom_command_topic->getValue());
  strcpy(ntpServer, custom_ntp_server->getValue());

  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["mqtt_server"] = mqttServer;
  json["mqtt_port"] = mqttPort;
  json["log_topic"] = logTopic;
  json["command_topic"] = commandTopic;
  json["ntp_server"] = ntpServer;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  } else {
    json.printTo(Serial);
    Serial.println();
    json.printTo(configFile);
    configFile.close();
  }
  //end save
  shouldSaveConfig = false;
}

void wifiManagerSetup() {
  //reset settings - for testing
  //wifiManager.resetSettings();

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
    
  //set custom ip for portal
  //wifiManager.setAPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //wifiManager.setRunServerAfterConnecting(true);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(custom_mqtt_server);
  wifiManager.addParameter(custom_mqtt_port);
  wifiManager.addParameter(custom_log_topic);
  wifiManager.addParameter(custom_command_topic);
  wifiManager.addParameter(custom_ntp_server);

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect("IoTClockConfig");
  //or use this for auto generated name ESP + ChipID
  //wifiManager.autoConnect();

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    saveConfig();
  }
}

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
  wifiManagerSetup();
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
    mqttClient.setServer(mqttServer, 1883);
    mqttClient.setCallback(callback);
  }
}

void wifiMaintainConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiBegin();
  } else {
    //wifiManager.loop();
    //save the custom parameters to FS
    if (shouldSaveConfig) {
      saveConfig();
    }
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
  Serial.print(mqttServer);
  Serial.print(" as ");
  Serial.println(mqttClientName);
  // this is still blocking, when a solution is found, mqttState of 1 will mean connecting.
  if (mqttClient.connect((char*) mqttClientName.c_str())) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    mqttClient.publish(logTopic, mqttClientName.c_str());
    // ... and resubscribe
    mqttClient.subscribe(commandTopic);
    Serial.print("Subscribed to: ");
    Serial.println(commandTopic);
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

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("Mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        Serial.println();
        if (json.success()) {
          Serial.println("parsed json");

          if (json.containsKey("mqtt_server")) strcpy(mqttServer, json["mqtt_server"]);
          if (json.containsKey("mqtt_port")) strcpy(mqttPort, json["mqtt_port"]);
          if (json.containsKey("log_topic")) strcpy(logTopic, json["log_topic"]);
          if (json.containsKey("command_topic")) strcpy(commandTopic, json["command_topic"]);
          if (json.containsKey("ntp_server")) strcpy(ntpServer, json["ntp_server"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    } else {
      Serial.println("/config.json not found");
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
  custom_mqtt_server = new WiFiManagerParameter("server", "MQTT Server", mqttServer, 40);
  custom_mqtt_port = new WiFiManagerParameter("port", "MQTT Port", mqttPort, 5);
  custom_log_topic = new WiFiManagerParameter("log", "MQTT Log Topic", logTopic, 60);
  custom_command_topic = new WiFiManagerParameter("command", "MQTT Command Topic", commandTopic, 60);
  custom_ntp_server = new WiFiManagerParameter("ntp", "NTP Server", ntpServer, 40);
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
  sendNTPpacket(ntpServer);
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
void sendNTPpacket(const char *address)
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

