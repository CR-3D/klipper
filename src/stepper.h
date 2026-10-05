#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h> // uint8_t

uint_fast8_t stepper_event(struct timer *t);

// Interface used by buffer_feed.c
struct stepper;
struct stepper *stepper_lookup_shared(uint8_t oid);
void stepper_inject_step(struct stepper *s);
uint_fast8_t stepper_get_dir_level(struct stepper *s);
uint_fast8_t stepper_set_idle_dir(struct stepper *s, uint_fast8_t level);

#endif // stepper.h
