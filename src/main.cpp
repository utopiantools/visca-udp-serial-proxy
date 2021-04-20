#include <Arduino.h>
#include <WiFi.h>
#include "AsyncUDP.h"
#include "M5Atom.h"

#include "visca.h"

/*
This code is designed for use with an M5Atom device with the RS-232 attachment

The biggest concern is that RS-232 uses different voltages than the TTL built in
to the Arduino boards, so an adapter chip is definitely needed.

Since the M5Atom devices have an RGB LED, it's also possible for us to
use the LED as a tally light.
*/

// uncomment this line if you are using an M5Atom Matrix
#define MATRIX

// copy settings.h.dist to settings.h and then modify settings.h
#include "settings.h"

// STATE VARIABLES
bool wifi_connected = false;
bool handling_long_press = false;
CAMSTATS cam_stats;

IPAddress ip;
int lastclientport = 0;
IPAddress lastclientip;
bool pwr_is_on = false;

// CLASS VARIABLES
WiFiClient tcp;    // we are a client for tally communication
WiFiClient remote; // remote client for receiving tcp commands
WiFiServer server(tcp_port);

AsyncUDP udp;
HardwareSerial viscaPort = Serial1;

// memory buffers for received VISCA commands
size_t lastudp_len = 0;
uint8_t lastudp_in[16];
size_t lastser_len = 0;
uint8_t lastser_in[16];

// quick color constants (red and green seem to be flipped ? )
const CRGB green = CRGB(0xf0, 0x00, 0x00);
const CRGB yellow = CRGB(0xf0, 0xf0, 0x00);
const CRGB blue = CRGB(0x00, 0x00, 0xf0);
const CRGB red = CRGB(0x00, 0xf0, 0x00);
const CRGB black = CRGB(0x00, 0x00, 0x00);
const CRGB purple = CRGB(0x00, 0xf0, 0xf0);
CRGB ledcolor = yellow;
CRGB oldledcolor = ledcolor;

// tally variables
int total_inputs = 2;          // vMix always loads with at least two inputs defined
int tally_input = 1;           // for mental consistency, we use input numbers based on 1 like vMix does
const int tally_bufsize = 100; // vMix can have a lot of inputs
uint8_t tb[tally_bufsize + 1]; // so there is always a null byte at the end... for printing

// remote control variables (not needed since we are using readString)
// const size_t rb_size = 32;
// uint8_t rb[rb_size];
String acc = "";

#define TALLY_SAFE 0
#define TALLY_LIVE 1
#define TALLY_PREVIEW 2

// all function definitions
// remember that C code doesn't do function culling
// code is processed from top to bottom, so
// you can't refer to a function before it is defined.

///
///
/// DEBUG FUNCTIONS
void debug();
void debug(char c);
void debug(int n, int base);
void debug(uint8_t *buf, int len);
void debug()
{
  Serial.println();
}
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
void debug(AsyncUDPPacket packet)
{
  Serial.print("Type of UDP datagram: ");
  Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast"
                                                                         : "Unicast");
  Serial.print(", Sender: ");
  Serial.print(packet.remoteIP());
  Serial.print(":");
  Serial.print(packet.remotePort());
  Serial.print(", Receiver: ");
  Serial.print(packet.localIP());
  Serial.print(":");
  Serial.print(packet.localPort());
  Serial.print(", Message length: ");
  Serial.print(packet.length());
  Serial.print(", Payload (hex):");
  debug(packet.data(), packet.length());
  debug();
}
void status()
{
  if (wifi_connected)
  {
    Serial.print("-- UDP LISTENING --\n");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.print(udp_port);
    debug();
    Serial.print("tx: G");
    Serial.print(txpin);
    Serial.print(" | rx: G");
    Serial.print(rxpin);
    debug();
  }
  else
  {
    Serial.print("CONNECTING ...\n");
    Serial.print(SSID);
    debug();
    Serial.print(PASSWORD);
    debug();
  }
}

void led_atom(CRGB c)
{
  M5.dis.drawpix(0, c);
}

void led_matrix(CRGB c)
{
  for (int i = 0; i < 25; i++)
    M5.dis.drawpix(i, c);
}

void led(CRGB c)
{
#ifdef MATRIX
  led_matrix(c);
#else
  led_atom(c);
#endif
  ledcolor = c;
}

void blink(CRGB c, int count = 2, int timeon = 50, int timeoff = 50)
{
  CRGB before = ledcolor;
  for (int i = 0; i < count; i++)
  {
    led(c);
    delay(timeon);
    led(before);
    delay(timeoff);
  }
}

///
///
/// VISCA TRANSLATION FUNCTIONS
/// these values are curved twice. one curve is based on the input velocity
/// the other curve is based on the camera's zoom / ptz position
double easeOutQuad(int x) { return 1 - (1 - x) * (1 - x); }
double easeOutCube(int x) { return 1 - pow(1 - x, 3); }
double easeOutCirc(int x) { return sqrt(1 - pow(x - 1, 2)); }
double ease(int x)
{
  if (CURVE == "CIRC")
    return easeOutCirc(x);
  if (CURVE == "QUAD")
    return easeOutQuad(x);
  if (CURVE == "CUBE")
    return easeOutCube(x);
  if (CURVE == "NONE")
    return x;
  // default
  return easeOutCirc(x);
}
double zoomcurve(int v) { return ZOOMMULT * pow(v, ZOOMEXP); }

// the closer we get the ZOOMMAX, the flatter the curve gets
double ptzcurve(int v) { return PTZMULT * pow(v, PTZEXP) * ease(cam_stats.z / ZOOMMAX); }
void handle_udp_visca(uint8_t *buf, size_t len)
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

  viscaPort.write(modified, lastelement + 1);
}
void handle_serial_visca(uint8_t *buf, size_t len){};

// check if we have received data over the serial port
void check_visca_serial()
{
  int available = viscaPort.available();
  if (available > 0)
  {
    CRGB oldcolor = ledcolor;
    led(yellow);
    while (available > 0)
    {
      Serial.println("Data available on Serial1");
      size_t actual = viscaPort.readBytesUntil(0xff, lastser_in, available); // does not include the terminator char
      if (actual < 16)
      {
        lastser_in[actual] = 0xff;
        actual++;
      }
      debug(lastser_in, actual);
      if (lastclientport > 0)
      {
        Serial.println("Sending to last UDP client.");
        udp.writeTo(lastser_in, actual, lastclientip, lastclientport);
      }
      debug();
      available = viscaPort.available();
    }
    led(oldcolor);
  }
}

void handle_packet(AsyncUDPPacket packet)
{
  CRGB oldc = ledcolor;
  led(yellow);

  lastclientip = packet.remoteIP();
  lastclientport = packet.remotePort();
  debug(packet);

  handle_udp_visca(packet.data(), packet.length());
  led(oldc);
}

void get_visca_status(int type) {}

///
///
/// UDP SERVER
void start_server()
{
  Serial.print("Starting UDP server on port: ");
  Serial.println(udp_port);
  udp.close(); // will close only if needed
  if (udp.listen(udp_port))
  {
    Serial.println("Server is Running!");
    udp.onPacket([](AsyncUDPPacket packet) {
      handle_packet(packet);
    });
  }
  else
  {
    Serial.println("Server failed to start");
  }

  Serial.print("Starting TCP server on port: ");
  Serial.println(tcp_port);
  server.begin();
}

///
///
/// TALLY FUNCTIONS
int tally_status(int input) { return tb[8 + input]; }
int tally_status() { return tally_status(tally_input); }
void update_tally_led()
{
  uint8_t ts = tally_status();
  debug(tb, tally_bufsize);
  debug('\n');
  debug(ts, HEX);
  debug('\n');
  switch (ts)
  {
  case TALLY_SAFE:
    led(black);
    break;
  case TALLY_LIVE:
    led(red);
    break;
  case TALLY_PREVIEW:
    led(green);
    break;
  }
}
void start_tally_tcp()
{
  led(black);
  Serial.println("Starting Tally Listener");
  tcp.connect(VMIX_IP, VMIX_PORT, 1000);
  if (tcp.connected())
    tcp.write("SUBSCRIBE TALLY\r\n");
  else
    blink(purple, 3, 100, 100);
}
void check_tally_tcp()
{
  int avail = tcp.available();
  if (avail == 0)
    return;

  Serial.println("TCP data in buffer!");
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
  size_t read = tcp.readBytesUntil('\n', tb, tally_bufsize); // never read more than the tally_bufsize
  if (read == 0)
    return;

  // we want to discard the \r at the end and make sure there is a null terminator
  tb[read - 1] = 0;

  // debug the ASCII data we got
  debug(tb, read - 1);
  Serial.println("");

  // convert the ASCII text to numerical values
  // ASCII NUMBERS 0-9 ARE ENCODED 48-57 (0x30-0x39)
  for (int i = 0; i < read; i++)
    tb[i] = (int)(tb[i] - 48);

  update_tally_led();
}

///
///
/// TCP SERVER HANDLER FOR UPDATING SETTINGS ON THE FLY
/// commands recognized are of the form VARIABLENAME=value\r\n
/// variables recognized are ZOOMMULT, ZOOMEXP, PTZMULT, PTZEXP
/// also variable can be CURVE and the values can be QUAD, CUBE, CIRC
void handle_incoming_tcp()
{
  String received = acc;
  acc = "";
  received = received.substring(0, received.length() - 2);
  Serial.println("RECEIVED TCP CONTROL DATA:");
  Serial.println(received);

  int idx = received.indexOf('=');
  if (idx > -1)
  {

    String var = received.substring(0, idx);
    var.toUpperCase();
    String val = received.substring(idx + 1); // ignore the \r
    float fval = val.toFloat();               // might be zero
    if (fval == 0)
      fval = 1;

    if (var == "STATUS")
    {
      remote.println("ZOOMMULT=" + String(ZOOMMULT));
      remote.println("ZOOMEXP=" + String(ZOOMEXP));
      remote.println("PTZMULT=" + String(PTZMULT));
      remote.println("PTZEXP=" + String(PTZEXP));
      remote.println("CURVE=" + String(CURVE));
      remote.println("CAMERA STATS ================");
      remote.println("       zoom: " + String(cam_stats.z, HEX));
      remote.println("     panpos: " + String(cam_stats.p));
      remote.println("    tiltpos: " + String(cam_stats.t));
      remote.println("      focus: " + String(cam_stats.f, HEX));
      remote.println("focus limit: " + String(cam_stats.flim, HEX));
    }
    else if (var == "POWER")
    {
      int onoff = fval;
      visca_power_sequence(onoff);
    }
    else if (var == "ZOOMMULT")
    {
      ZOOMMULT = fval;
    }
    else if (var == "ZOOMEXP")
    {
      ZOOMEXP = fval;
    }
    else if (var == "PTZMULT")
    {
      PTZMULT = fval;
    }
    else if (var == "PTZEXP")
    {
      PTZEXP = fval;
    }
    else if (var == "CURVE")
    {
      if (val == "NONE" || val == "QUAD" || val == "CUBE" || val == "CIRC")
      {
        CURVE = val;
      }
      else
      {
        remote.print("UNKNOWN CURVE: " + val + ". (MUST BE NONE, QUAD, CUBE, OR CIRC)\r\n");
        return;
      }
    }
    else
    {
      remote.println(received + " - COMMAND DOES NOT EXIST");
      return;
    }
    remote.println(received + " - SUCCESS");
  }
  else
  {
    remote.print("COULD NOT UNDERSTAND: " + received + "\r\n");
    return;
  }
}
void check_remote_incoming_tcp()
{
  while (remote.available() > 0)
  {
    char c = remote.read();
    acc += c;
    if (acc.endsWith("\n"))
    {
      handle_incoming_tcp();
    }
  }
}
void check_server_tcp()
{
  if (remote && remote.connected())
  {
    check_remote_incoming_tcp();
  }
  else
  {
    remote = server.available();
    if (remote && remote.connected())
    {
      remote.flush();
      acc = "";
      Serial.println("TCP client connected");
      remote.write("VISCA PROXY TCP CONTROL... HERE ARE YOUR INSTRUCTIONS:\r\n---------------------------------------------");
      remote.write("\r\nENTER A COMMAND LIKE THIS: VARIABLE=VALUE\\r\\n\r\n");
      remote.write("FLOAT VARIABLES: ZOOMMULT, ZOOMEXP, PTZMULT, PTZEXP\r\n");
      remote.write("STRING VARIABLE: CURVE (NONE, QUAD, CUBE, CIRC) slows movements depending on zoom level\r\n");
      remote.write("COMMAND: STATUS=1 (RETURNS STATUS DATA)\r\n");
      remote.write("COMMAND: POWER=(0,1) (TURNS THE CAMERA ON OR OFF)\r\n");
      check_remote_incoming_tcp();
    }
  }
}

///
///
/// WIFI CALLBACKS
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
  led(green);
  ip = info.got_ip.ip_info.ip.addr;
  Serial.println("WiFi Connected!");
  Serial.println(ip);
  wifi_connected = true;
  start_server(); // will stop any previous servers
  // start_tally_tcp();
}
void WiFiDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  ip = IPAddress(0, 0, 0, 0);
  wifi_connected = false;
}

///
///
/// SETUP AND LOOP
void setup()
{
  M5.begin(true, true, true);
  Serial.begin(baudrate);

  led(yellow);

  Serial.println("started...");
  Serial.println("connecting to WiFi...");

  // connect to WiFi
  WiFi.setAutoConnect(true);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(WiFiDisconnected, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);
  WiFi.begin(SSID, PASSWORD, 0, NULL, true);

  // start the visca serial connection
  Serial.println("connecting to camera...");
  viscaPort.begin(baudrate, SERIAL_8N1, rxpin, txpin, false, 200);

  visca_init(&viscaPort);
  visca_power_sequence(true);
}

int loopcounter = 0;
void loop()
{
  // Serial.print('.');

  // blink if no wifi connection
  if (!wifi_connected)
  {
    Serial.println("connecting...");
    led(black);
    delay(100);
    led(yellow);
    delay(100);
    return;
  }

  if (M5.Btn.isPressed())
  {
    oldledcolor = ledcolor == purple ? oldledcolor : ledcolor;
    led(purple);
  }
  // GET USER INPUT
  if (M5.Btn.pressedFor(750) && !handling_long_press)
  {
    handling_long_press = true;
    Serial.println("long press button!");
    // find the next active tally by cycling through all the inputs to see
    // if any of them are preview
    for (int i = 0; i < total_inputs; i++)
    {
      tally_input = (tally_input % total_inputs) + 1; // mod before add yields 1-indexed values
      if (tally_status() == 2)
        break;
    }
    Serial.print("WATCHING INPUT: ");
    Serial.println(tally_input);
    update_tally_led();
    if (!tcp.connected())
      start_tally_tcp();
  }
  else if (M5.Btn.wasReleased())
  {
    led(oldledcolor);
    if (!handling_long_press)
    {
      tally_input = (tally_input % total_inputs) + 1; // mod before add yields 1-indexed values
      Serial.println("button!");
      Serial.print("WATCHING INPUT: ");
      Serial.println(tally_input);
      update_tally_led();
      if (!tcp.connected())
        start_tally_tcp();
    }
    handling_long_press = false;
  }

  check_visca_serial();
  check_tally_tcp();
  check_server_tcp();

  // recheck the visca stats every 500 ms
  loopcounter = (loopcounter + 1) % 50;
  switch (loopcounter)
  {
  case 0:
    cam_stats = visca_get_stats();
    break;
  }

  // don't overload the CPU! take a breather
  delay(10);

  M5.update(); // reads the button again among other things.
}
