#ifndef __C5_LEVELBOARD_H
#define __C5_LEVELBOARD_H

#include <stdint.h>

void c5_levelboard_capture(uint16_t count);
uint8_t c5_levelboard_eddy_state(void);
void c5_levelboard_cancel(void);
void c5_levelboard_recover(void);

#endif // c5_levelboard.h
