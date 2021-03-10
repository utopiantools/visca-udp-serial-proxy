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
WiFiClient tcp;
AsyncUDP udp;
int lastclientport = 0;
IPAddress lastclientip;
bool pwr_is_on = false;

// memory buffers for VISCA commands
size_t lastudp_len = 0;
uint8_t lastudp_in[16];
size_t lastser_len = 0;
uint8_t lastser_in[16];

// quick use VISCA commands
const uint8_t pwr_on[] = {0x81, 0x01, 0x04, 0x00, 0x02, 0xff};
const uint8_t pwr_off[] = {0x81, 0x01, 0x04, 0x00, 0x03, 0xff};
const uint8_t addr_set[] = {0x88, 0x30, 0x01, 0xff};       // address set
const uint8_t if_clear[] = {0x88, 0x01, 0x00, 0x01, 0xff}; // if clear

// quick color constants
const CRGB green = CRGB(0xf0, 0x00, 0x00);
const CRGB yellow = CRGB(0xf0, 0xf0, 0x00);
const CRGB blue = CRGB(0x00, 0x00, 0xf0);
const CRGB red = CRGB(0x00, 0xf0, 0x00);
const CRGB black = CRGB(0x00, 0x00, 0x00);
CRGB ledcolor = yellow;

// tally variables
int total_inputs = 2;          // vMix always loads with at least two inputs defined
int tally_input = 1;           // for mental consistency, we use input numbers based on 1 like vMix does
int tally_status = 0;          // 0: safe, 1: live, 2: preview
const int tally_bufsize = 100; // vMix can have a lot of inputs
uint8_t tb[tally_bufsize + 1]; // so there is always a null byte at the end... for printing

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

void visca_power(bool turnon)
{
  if (turnon)
  {
    send_visca(addr_set);
    delay(500);
    send_visca(pwr_on);
    delay(2000);
    send_visca(if_clear);
  }
  else
  {
    send_visca(if_clear);
    delay(2000);
    send_visca(pwr_off);
  }
  pwr_is_on = turnon;
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
    Serial.println(udp_port);
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

void start_server()
{
  udp.close(); // will close only if needed
  if (udp.listen(udp_port))
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

int read_tally(int input) { return (tb[8 + input]) - 48; }
int read_tally() { return read_tally(tally_input); }

void update_tally()
{
  tally_status = read_tally(); // ASCII NUMBERS 0-9 ARE ENCODED 48-57 (0x30-0x39)

  switch (tally_status)
  {
  case 0:
    led(black);
    break;
  case 1:
    led(red);
    break;
  case 2:
    led(green);
    break;
  }
}

void start_tally()
{
  tcp.connect(VMIX_IP, VMIX_PORT);
  tcp.write("SUBSCRIBE TALLY\r\n");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
  led(green);
  ip = info.got_ip.ip_info.ip.addr;
  wifi_connected = true;
  start_tally();
  start_server(); // will stop any previous servers
}

void WiFiDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  ip = IPAddress(0, 0, 0, 0);
  wifi_connected = false;
}

// check if we have received data over the serial port
void check_serial()
{
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
    update_tally();
  }
}

void check_tally()
{
  int avail = tcp.available();
  if (avail == 0)
    return;

  // TALLY DATA ALWAYS LOOKS LIKE THIS:
  // TALLY OK 120001\r\n
  // (encoded as ASCII)
  // input x will be at buffer index 8 + x (because inputs are 1-based)
  // total inputs will be avail - 9
  int inputs = avail - 11; // the data will be terminated with \r\n
  if (inputs > 1 && total_inputs != inputs)
  {
    Serial.print("INPUT COUNT HAS CHANGED TO: ");
    Serial.println(inputs);
    total_inputs = inputs;
  }

  // vMix terminates messages with \r\n
  // readBytesUntil will discard the terminator char
  // but we also want to discard the \r before it
  size_t read = tcp.readBytesUntil('\n', tb, tally_bufsize); // never read more than the tally_bufsize
  if (read == 0)
    return;

  debug(tb, read);
  Serial.println("");
  tb[read - 1] = 0;           // convert the \r to 0 or at least make sure the final char has a null
  Serial.println((char *)tb); // since there's a null at the end, we can print the buffer like a string

  update_tally();
}

// put your setup code here, to run once:
void setup()
{
  M5.begin(true, false, true);
  led(yellow);

  Serial.begin(baudrate);
  Serial.println("started...");
  Serial.println("connecting to WiFi...");

  // connect to WiFi
  WiFi.setAutoConnect(true);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(WiFiDisconnected, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);
  WiFi.begin(SSID, PASSWORD, 0, NULL, true);

  // start the visca serial connection
  Serial.println("connecting to camera...");
  Serial1.begin(baudrate, SERIAL_8N1, rxpin, txpin, false, 200);

  visca_power(true);
}

void loop()
{
  if (M5.Btn.wasPressed())
  {
    tally_input = (tally_input % total_inputs) + 1; // mod before add yields 1-indexed values
    Serial.println("button!");
    Serial.print("WATCHING INPUT: ");
    Serial.println(tally_input);
    update_tally();
  }
  else if (M5.Btn.pressedFor(750))
  {
    start_tally();
    Serial.println("long press button!");
    // find the next active tally by cycling through all the inputs to see
    // if any of them are preview
    for (int i = 0; i < total_inputs; i++)
    {
      tally_input = (tally_input % total_inputs) + 1; // mod before add yields 1-indexed values
      if (read_tally() == 2)
        break;
    }
    Serial.print("WATCHING INPUT: ");
    Serial.println(tally_input);
    update_tally();
  }

  // blink if no wifi connection
  if (!wifi_connected)
  {
    led(black);
    delay(100);
    led(yellow);
    delay(100);
    return;
  }

  check_serial();
  check_tally();

  // check if we have received data over the tally tcp port
  // later...

  // don't overload the CPU! take a breather
  delay(20);
  M5.update(); // reads the button again among other things.
}
