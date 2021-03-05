#include <Arduino.h>
#include <WiFi.h>
#include "AsyncUDP.h"
#include "M5Atom.h"

/*
This code is designed for use with an M5Atom device with the RS-232 attachment

The biggest concern is that RS-232 uses different voltages than the TTL built in
to the Arduino boards, so an adapter chip is definitely needed.

Since the M5Atom devices have an RGB LED, it's also possible for us to
use the LED as a tally light.
*/

// uncomment this line if you are using an M5Atom Matrix
// #define MATRIX

// copy settings.h.dist to settings.h and then modify settings.h
#include "settings.h"

// STATE VARIABLES
bool wifi_connected = false;

IPAddress ip;
AsyncUDP udp;
int port = 52381;
int lastclientport = 0;
IPAddress lastclientip;

// memory buffers for VISCA commands
size_t lastudp_len = 0;
uint8_t lastudp_in[16];
size_t lastser_len = 0;
uint8_t lastser_in[16];

// quick use VISCA commands
uint8_t pwr_on[] = {0x81, 0x01, 0x04, 0x00, 0x02, 0xff};
uint8_t pwr_off[] = {0x81, 0x01, 0x04, 0x00, 0x03, 0xff};
uint8_t addr_set[] = {0x88, 0x30, 0x01, 0xff};       // address set
uint8_t if_clear[] = {0x88, 0x01, 0x00, 0x01, 0xff}; // if clear

// quick color commands
CRGB red = CRGB(0xf0, 0x00, 0x00);
CRGB yellow = CRGB(0xf0, 0xf0, 0x00);
CRGB blue = CRGB(0x00, 0x00, 0xf0);
CRGB green = CRGB(0x00, 0xf0, 0x00);
CRGB black = CRGB(0x00, 0x00, 0x00);
CRGB ledcolor = yellow;
int tallystate = 0;

// all function definitions
// remember that C code doesn't do function culling
// code is processed from top to bottom, so
// you can't refer to a function before it is defined.

void debug(char c);
void debug(int n, int base);
void debug(uint8_t *buf, int len);
void send_bytes(uint8_t *b, int len);

void debug(char c)
{
  Serial.print(c);
}
void debug(int n, int base)
{
  Serial.print(n, base);
  Serial.print(' ');
}

void debug(uint8_t *buf, int len)
{
  for (uint8_t i = 0; i < len; i++)
  {
    uint8_t elem = buf[i];
    debug(elem, HEX);
  }
}

void send_bytes(uint8_t *b, int len)
{
  for (int i = 0; i < len; i++)
  {
    uint8_t elem = b[i];
    debug(elem);
    Serial1.write(elem);
  }
}

void send_visca(uint8_t *c, size_t len)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    Serial1.write(elem);
  } while (i < len && elem != 0xff);
  Serial.println("");
}

// the buffer MUST end with a 0xff
void send_visca(const uint8_t *c)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    Serial1.write(elem);
  } while (elem != 0xff);
  Serial.println("");
}

double zoomcurve(int v)
{
  return ZOOMMULT * pow(v, ZOOMEXP);
}

double ptzcurve(int v)
{
  return PTZMULT * pow(v, PTZEXP);
}

void handle_visca(uint8_t *buf, size_t len)
{
  uint8_t modified[16];
  size_t lastelement = 0;
  for (int i = 0; (i < len && i < 16); i++)
  {
    modified[i] = buf[i];
    lastelement = i;
  }

  // is this a PTZ?
  if (modified[1] == 0x01 && modified[2] == 0x06 && modified[3] == 0x01)
  {
    Serial.println("PTZ CONTROL DETECTED... ADJUSTING SPEED");
    modified[4] = (int)ptzcurve(modified[4]);
    modified[5] = (int)ptzcurve(modified[5]);
  }
  if (modified[1] == 0x01 && modified[2] == 0x04 && modified[3] == 0x07)
  {
    Serial.println("ZOOM CONTROL DETECTED, ADJUSTING SPEED");
    int zoomspeed = modified[4] & 0b00001111;
    zoomspeed = (int)zoomcurve(zoomspeed);
    int zoomval = (modified[4] & 0b11110000) + zoomspeed;
    modified[4] = zoomval;
  }

  Serial1.write(modified, lastelement + 1);
}

void status()
{
  if (wifi_connected)
  {
    Serial.println("-- UDP LISTENING --");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(port);
    Serial.print("tx: G");
    Serial.print(txpin);
    Serial.print(" | rx: G");
    Serial.println(rxpin);
  }
  else
  {
    Serial.println("CONNECTING ...");
    Serial.println(SSID);
    Serial.println(PASSWORD);
  }
}

void led(CRGB c)
{
  M5.dis.drawpix(0, c);
  ledcolor = c;
}

void tally(int state)
{
  switch (state)
  {
  case 0:
    led(black);
    break;
  case 1:
    led(red);
    break;
  case 2:
    led(blue);
    break;
  }
  tallystate = state;
}
void tally()
{
  tally(tallystate);
}

void start_server()
{
  udp.close(); // will close only if needed
  if (udp.listen(port))
  {
    udp.onPacket([](AsyncUDPPacket packet) {
      CRGB oldc = ledcolor;
      led(yellow);

      // debug(packet);
      lastclientip = packet.remoteIP();
      lastclientport = packet.remotePort();

      Serial.print("Type of UDP datagram: ");
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast"
                                                                             : "Unicast");
      Serial.print(", Sender: ");
      Serial.print(lastclientip);
      Serial.print(":");
      Serial.print(lastclientport);
      Serial.print(", Receiver: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Message length: ");
      Serial.print(packet.length());
      Serial.print(", Payload (hex):");
      debug(packet.data(), packet.length());
      Serial.println();

      handle_visca(packet.data(), packet.length());
      led(oldc);
    });
  }
}

void wait_connection()
{
  while (!wifi_connected)
  {
    wifi_connected = WiFi.isConnected();
    status();
    delay(1000);
  }
  start_server();
}

// put your setup code here, to run once:
void setup()
{
  M5.begin(true, false, true);
  led(yellow);

  Serial.begin(baudrate);
  Serial.println("started...");
  Serial.println("connecting to WiFi...");

  // start the visca serial connection
  Serial1.begin(baudrate, SERIAL_8N1, rxpin, txpin, false, 200);

  // connect to WiFi
  WiFi.setAutoConnect(true);
  WiFi.begin(SSID, PASSWORD, 0, NULL, true);

  send_visca(addr_set);
  send_visca(pwr_on);

  M5.dis.drawpix(0, CRGB(0xf0, 0xf0, 0x00));
}

void loop()
{

  if (M5.Btn.wasPressed())
  {
    Serial.println("button!");
    Serial.println(tallystate);
    tally((tallystate + 1) % 3);
  }

  // check for wifi connection
  bool nowconnected = WiFi.isConnected();
  if (!nowconnected)
  {
    led(yellow);
    wifi_connected = false;
    wait_connection();
    tally(0);
  }

  // check if we have received data over the serial port
  int available = Serial1.available();
  while (available > 0)
  {
    led(yellow);
    Serial.println("Data available on Serial1");
    int actual = Serial1.readBytesUntil(0xff, lastser_in, available); // does not include the terminator char
    if (actual < 16)
    {
      lastser_in[actual] = 0xff;
      actual++;
    }
    debug(lastser_in, actual);
    if (lastclientport > 0)
      udp.writeTo(lastser_in, actual, lastclientip, lastclientport);
    Serial.println("");
    available = Serial1.available();
    tally();
  }

  // check if we have received data over the tally tcp port
  // later...

  // don't overload the CPU! take a breather
  delay(20);
  M5.update(); // reads the button again among other things.
}