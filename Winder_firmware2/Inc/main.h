#ifndef MAIN_H
#define MAIN_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * HAL Handle Declarations
 * These are initialized by HAL during MX_* functions.
 * ----------------------------------------------------------------------- */
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim2;   /* Stepper STEP pulses */
extern TIM_HandleTypeDef  htim3;   /* DC motor PWM        */

/* -----------------------------------------------------------------------
 * Winder Application API
 * ----------------------------------------------------------------------- */

/**
 * @brief Initialize winder subsystems (encoders, drivers, control state).
 *        Call once at startup before entering main loop.
 */
void Winder_Init(void);

/**
 * @brief Start a winding run (transitions from IDLE to PRETENSION state).
 */
void Winder_Start(void);

/**
 * @brief Control loop — call at 1kHz from main() or SysTick ISR.
 *        Reads sensors, updates state machine, controls motors.
 */
void Winder_ControlLoop(void);

/**
 * @brief Control tick flag — set to true by SysTick interrupt at 1kHz.
 *        Main loop clears it after calling Winder_ControlLoop().
 */
extern volatile bool control_tick;

/* -----------------------------------------------------------------------
 * TIM2 IRQ Handler — stepper pulse counting
 * Add this to stm32f3xx_it.c if not already present.
 * ----------------------------------------------------------------------- */
void TIM2_IRQHandler(void);

#endif /* MAIN_H */
