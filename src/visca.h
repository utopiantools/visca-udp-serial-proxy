#include <M5Atom.h>

void visca_init(HardwareSerial *port);
void visca_send(const uint8_t *visca_bytes);
void visca_power_sequence(bool turnon);