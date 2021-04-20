#include <M5Atom.h>

struct CAMSTATS
{
	uint16_t z = 0;			 // zoom position (sony optical max is 0x4000)
	uint16_t f = 0x1000; // focus position (inf is 0x1000)
	uint16_t flim = 0;	 // focus near limit
	int16_t p = 0;			 // pan position (signed integer)
	int16_t t = 0;			 // tilt position (signed integer)
};

void visca_init(HardwareSerial *port);
void visca_send(const uint8_t *visca_bytes);
void visca_power_sequence(bool turnon);
CAMSTATS visca_get_stats();
