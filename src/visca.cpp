#include "visca.h"

// quick use VISCA commands
const uint8_t pwr_on[] = {0x81, 0x01, 0x04, 0x00, 0x02, 0xff};
const uint8_t pwr_off[] = {0x81, 0x01, 0x04, 0x00, 0x03, 0xff};
const uint8_t addr_set[] = {0x88, 0x30, 0x01, 0xff};			 // address set
const uint8_t if_clear[] = {0x88, 0x01, 0x00, 0x01, 0xff}; // if clear

/// INQUIRIES

const uint8_t cam_lens_inq[] = {0x81, 0x09, 0x7E, 0x7E, 0x00, 0xff};
// RESPONSE:
// 0w 0w 0w 0w 0v 0v 0y 0y 0y 0y 00 WW VV
// w: zoom position
// v: focus near limit
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

HardwareSerial *_viscaPort = 0;
void visca_init(HardwareSerial *port) { _viscaPort = port; }

// the buffer MUST end with a 0xff
void visca_send(const uint8_t *c)
{
	if (_viscaPort == 0)
		return;
	int i = 0;
	uint8_t elem;
	do
	{
		elem = c[i++];
		_viscaPort->write(elem);
	} while (elem != 0xff);
}

void visca_power_sequence(bool turnon)
{
	if (turnon)
	{
		visca_send(addr_set);
		delay(500);
		visca_send(pwr_on);
		delay(2000);
		visca_send(if_clear);
	}
	else
	{
		visca_send(if_clear);
		delay(2000);
		visca_send(pwr_off);
	}
}

void visca_get_zoom()
{
}
