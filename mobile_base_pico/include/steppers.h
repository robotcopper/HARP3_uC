/*
 * The three stepper drivers: ENABLE, DIR, and STEP from a PWM slice.
 */

#ifndef MOBILE_BASE_STEPPERS_H
#define MOBILE_BASE_STEPPERS_H

#include <stdbool.h>
#include <stdint.h>

#include "motion.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Leaves the drivers disabled. A timer interrupt stops the pulses once no
   command has arrived for timeout_ms, whatever the application task is doing. */
void steppers_init(uint32_t timeout_ms);

/* Disabling also stops the pulses, and commands are ignored until enabled. */
void steppers_enable(bool enable);

/* Callable from any task. False while disabled. */
bool steppers_command(const double wheel_velocity[WHEEL_COUNT]);

/* True while at least one wheel receives pulses. */
bool steppers_running(void);

#ifdef __cplusplus
}
#endif

#endif /* MOBILE_BASE_STEPPERS_H */
