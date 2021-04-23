#include "visca.h"

byte _vb[16];

// quick use VISCA commands
const uint8_t pwr_on[] = {0x81, 0x01, 0x04, 0x00, 0x02, 0xff};
const uint8_t pwr_off[] = {0x81, 0x01, 0x04, 0x00, 0x03, 0xff};
const uint8_t addr_set[] = {0x88, 0x30, 0x01, 0xff};	   // address set
const uint8_t if_clear[] = {0x88, 0x01, 0x00, 0x01, 0xff}; // if clear

/// INQUIRIES
const uint8_t cam_lens_inq[] = {0x81, 0x09, 0x7E, 0x7E, 0x00, 0xff};
// PACKET DATA (after the 0x90 0x50):
// 0w 0w 0w 0w 0v 0v 0y 0y 0y 0y 00 WW VV
// w: zoom position
// v: focus near limit (bit shift 8 more bits)
// y: focus position
// WW:
//     bit 0 indicates autofocus status,
//     bit 1 indicates digital zoom status
//     bit 2 indicates AF sensitivity / 0-slow 1-normal
//     bits 3-4 indicate AF mode / 0-normal, 1-interval, 2-zoom trigger
// VV:
//     bit 0 indicates zooming status / 0-stopped, 1-executing
//     bit 1 indicates focusing status
//     bit 2 indicates camera memory recall status / 0-stopped, 1-executing
//     bit 3 indicates low contrast detection

const uint8_t cam_img_inq[] = {0x81, 0x09, 0x7E, 0x7E, 0x01, 0xff};
// PACKET DATA (after the 0x90 0x50):
// 0w 0w 0v 0v 0a 0b 0c AA BB CC DD EE FF
// w: R gain
// v: B gain
// a: WB mode
// b: aperture gain
// c: exposure mode
// AA:
//     bit 0 slow shutter / 1-auto, 0-manual
//     bit 1 exposure comp
//     bit 2 backlight
//     bit 3 unused
//     bit 4 wide D / 0-off, 1-other
//     bit 5 High Res
// BB: shutter position
// CC: iris position
// DD: gain position
// EE: brightness
// FF: exposure

const uint8_t ptz_pos_inq[] = {0x81, 0x09, 0x06, 0x12, 0xff};
//RESPONDS: 90 50 0w 0w 0w 0w 0z 0z 0z 0z FF
// wwww: pan position (signed)
// zzzz: tilt position (signed)

HardwareSerial *_viscaPort = 0;

CAMSTATS stats;

void visca_init(HardwareSerial *port) { _viscaPort = port; }

// the buffer MUST end with a 0xff
void visca_send(const uint8_t *c)
{
	Serial.print("Sending VISCA: ");
	if (_viscaPort == 0)
		return;
	int i = 0;
	uint8_t elem;
	do
	{
		elem = c[i++];
		Serial.print(elem, HEX);
		Serial.print(" ");
		_viscaPort->write(elem);
	} while (elem != 0xff);
	Serial.println();
}

void visca_power_sequence(bool turnon)
{
	if (turnon)
	{
		visca_send(addr_set);
		delay(10);
		visca_send(pwr_on);
		delay(2000);
		visca_send(if_clear);
	}
	else
	{
		visca_send(if_clear);
		delay(10);
		visca_send(pwr_off);
	}
}

CAMSTATS visca_get_stats(int mode)
{
	if (mode == 0)
	{
		visca_send(cam_lens_inq);
		// expected reply will be 16 bytes
		// 90 50 0p 0q 0r 0s 0v 0v 0y 0y 0y 0y 00 WW VV FF
		// pqrs: Zoom Position
		delay(50);
		if (_viscaPort->available() == 0)
			return stats;
		int read = _viscaPort->readBytesUntil(0xff, _vb, 16); // doesn't include terminator byte
		if (read < 15)
			return stats;

		stats.z = (_vb[2] << 12) | (_vb[3] << 8) | (_vb[4] << 4) | _vb[5];	 // 0x4000 is the max optical zoom for Sony cameras
		stats.flim = (_vb[6] << 12) | (_vb[7] << 8);						 // near limit is 0x1000 (inf) - 0xF000;
		stats.f = (_vb[8] << 12) | (_vb[9] << 8) | (_vb[10] << 4) | _vb[11]; // 0x1000 is far / inf
		Serial.println("zoom stats received");
	}
	else if (mode == 1)
	{
		visca_send(ptz_pos_inq);
		// expected reply will be 11 bytes
		// 90 50 0w 0w 0w 0w 0z 0z 0z 0z FF
		// wwww: PAN Position (signed int) zzzz: Tilt Position (signed)
		delay(50);
		if (_viscaPort->available() == 0)
			return stats;
		int read = _viscaPort->readBytesUntil(0xff, _vb, 11); // doesn't include terminator byte
		if (read < 10)
			return stats;

		stats.p = (_vb[2] << 12) | (_vb[3] << 8) | (_vb[4] << 4) | _vb[5]; // the bytes will be cast to a signed value
		stats.t = (_vb[6] << 12) | (_vb[7] << 8) | (_vb[8] << 4) | _vb[9]; // the bytes will be cast to a signed value
		Serial.println("ptz pos stats received");
	}

	_viscaPort->flush();
	return stats;
}
