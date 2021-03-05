#include <Arduino.h>
#include <WiFi.h>
#include "AsyncUDP.h"
#include "M5Atom.h"

#define OFF HIGH
#define ON LOW
#define MATRIX

#define PRINT(X) \
  Serial.print(X);
#define PRINTLN(X) \
  Serial.println(X);

// const char *SSID = "MIKELS_2.4";
// const char *PASSWORD = "in the beginning was the word";
const char *SSID = "LCC AV";
const char *PASSWORD = "#NerdAlert";
const int port = 52381;
const int txpin = 19; //26
const int rxpin = 22; //36
const int led = 10;
const int baudrate = 38400;

float ZOOMMULT = 0.3;
float PTZMULT = 0.3;

bool connected = false;
IPAddress ip;
AsyncUDP udp;
AsyncUDPPacket *lastUDPpacket;
IPAddress lastclientip;
int lastclientport = 0;

uint8_t pwr_on[] = {0x81, 0x01, 0x04, 0x00, 0x02, 0xff};
uint8_t addr_set[] = {0x88, 0x30, 0x01, 0xff};       // address set
uint8_t if_clear[] = {0x88, 0x01, 0x00, 0x01, 0xff}; // if clear

size_t lastudp_len = 0;
uint8_t lastudp_in[16];
size_t lastser_len = 0;
uint8_t lastser_in[16];

void debug(int elem, int base)
{
  Serial.print(elem, base);
  Serial.print(' ');
}

void debug(char c)
{
  Serial.print(c);
}

void debug(uint8_t *c, int len)
{
  for (uint8_t i = 0; i < len; i++)
  {
    uint8_t elem = c[i];
    debug(elem, HEX);
  }
}

void send(uint8_t b)
{
  debug(b);
  Serial1.write(b);
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
    send(elem);
  } while (i < len && elem != 0xff);
  PRINTLN("");
}

void send_visca(uint8_t *c)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    send(elem);
  } while (elem != 0xff);
  PRINTLN("");
}

double zoomcurve(int v)
{
  return ZOOMMULT * pow(v, 1.5);
}

double ptzcurve(int v)
{
  return v * PTZMULT;
}

void status()
{
  if (connected)
  {
    PRINTLN("-- UDP LISTENING --");
    PRINT(WiFi.localIP());
    PRINT(":");
    PRINTLN(port);
    PRINT("tx: G");
    PRINT(txpin);
    PRINT(" | rx: G");
    PRINTLN(rxpin);
  }
  else
  {
    PRINTLN("CONNECTING ...");
    PRINTLN(SSID);
    PRINTLN(PASSWORD);
  }
}

void handleVisca(uint8_t *buf, size_t len)
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
    PRINTLN("PTZ CONTROL DETECTED... ADJUSTING SPEED");
    modified[4] = (int)ptzcurve(modified[4]);
    modified[5] = (int)ptzcurve(modified[5]);
  }
  if (modified[1] == 0x01 && modified[2] == 0x04 && modified[3] == 0x07)
  {
    PRINTLN("ZOOM CONTROL DETECTED, ADJUSTING SPEED");
    int zoomspeed = modified[4] & 0b00001111;
    zoomspeed = (int)zoomcurve(zoomspeed);
    int zoomval = (modified[4] & 0b11110000) + zoomspeed;
    modified[4] = zoomval;
  }

  Serial1.write(modified, lastelement + 1);
}

void startServer()
{
  udp.close(); // will close only if needed
  if (udp.listen(port))
  {
    udp.onPacket([](AsyncUDPPacket packet) {
      digitalWrite(led, LOW);
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

      // pass the data through to the GPIO serial pins
      handleVisca(packet.data(), packet.length());
      // lastUDPpacket = &packet;
      // digitalWrite(led, HIGH);
      // status();
    });
  }
}

void waitConnection()
{
  while (!connected)
  {
    connected = WiFi.isConnected();
    status();
    delay(1000);
  }
  startServer();
}

// put your setup code here, to run once:
void setup()
{
  M5.begin(true, false, true);

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
}

void loop()
{
  // check for wifi connection
  bool nowconnected = WiFi.isConnected();
  if (!nowconnected)
  {
    connected = false;
    waitConnection();
  }

  // check if we have received data over the serial port
  int available = Serial1.available();
  while (available > 0)
  {
    PRINTLN("Data available on Serial1");
    int actual = Serial1.readBytesUntil(0xff, lastser_in, available); // does not include the terminator char
    if (actual < 16)
    {
      lastser_in[actual] = 0xff;
      actual++;
    }
    debug(lastser_in, actual);
    if (lastclientport > 0)
      udp.writeTo(lastser_in, actual, lastclientip, lastclientport);
    PRINTLN("");
    available = Serial1.available();
  }

  // don't overload the CPU! take a breather
  delay(20);
}