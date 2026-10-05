#ifndef SIM_MISC_H
#define SIM_MISC_H
#include <stdint.h>
uint32_t timer_read_time(void);
static inline uint32_t timer_from_us(uint32_t us) { return us * 64; }
static inline int timer_is_before(uint32_t a, uint32_t b) {
    return (int32_t)(a - b) < 0;
}
#endif
