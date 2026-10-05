#ifndef __C5_EBOARD_H
#define __C5_EBOARD_H

#include <stdint.h>

void c5_eboard_capture(uint16_t interval);
uint8_t c5_eboard_eddy_state(void);
void c5_eboard_arm(void);
void c5_eboard_pa_receive(volatile uint8_t *frame, uint16_t remaining);
void c5_eboard_set_pa_mode(uint8_t active);
uint8_t c5_eboard_pa_matches(const volatile int32_t *samples, uint16_t count);
void c5_eboard_init(void);

#endif // c5_eboard.h
