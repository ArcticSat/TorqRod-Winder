#ifndef STEPPER_H
#define STEPPER_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * Stepper Driver — NEMA 23 via STEP/DIR
 *
 * Motor:   Pololu 1472, 200 steps/rev
 * Driver:  Pololu DRV8434A, 1/16 microstep → 3200 steps/rev
 * Timer:   TIM2 CH1 in PWM mode — hardware-generated STEP pulses
 *          Prescaler = 63 → 1MHz timer clock
 *          ARR = (1,000,000 / step_hz) - 1    sets pulse rate
 *          CCR1 = ARR / 2                      50% duty, ensures ≥1µs pulse
 *
 * The speed controller output is omega_target (rad/s spindle shaft).
 * This driver converts that to step_rate_hz and updates ARR/CCR1 atomically.
 *
 * Ramp: applied externally by the speed controller (5.24 rad/s² limit).
 * ----------------------------------------------------------------------- */

/* Motor parameters */
#define STEPPER_STEPS_PER_REV   200U
#define STEPPER_MICROSTEP       16U
#define STEPPER_USTEPS_PER_REV  (STEPPER_STEPS_PER_REV * STEPPER_MICROSTEP)  /* 3200 */

/* Timer clock after prescaler (1MHz) */
#define STEPPER_TIMER_CLK_HZ    1000000UL

/* Speed limits */
#define STEPPER_MAX_OMEGA_RAD_S  8.0f    /* ~76 RPM — well within motor limits */
#define STEPPER_MIN_OMEGA_RAD_S  0.01f   /* Below this: hold (timer stopped)   */

/* Minimum ARR to prevent over-driving the step input (~40kHz max for TMC2208) */
#define STEPPER_MIN_ARR         24U      /* 1MHz / 25 ≈ 40kHz */
#define STEPPER_MAX_ARR         0xFFFFUL /* 16-bit TIM2 on F303 — ~15 steps/s  */

/* DRV8434A control interface:
 * STEP/DIR for motion, ENN for enable.
 * ENN pin: active LOW. Pull LOW to enable, HIGH to disable (coast).
 * DIR: controlled via GPIO, HIGH = forward (increasing layer count), LOW = reverse.
 */

/* GPIO convenience — match your CubeMX-generated defines */
#define STEPPER_DIR_PIN     GPIO_PIN_1
#define STEPPER_DIR_PORT    GPIOA
#define STEPPER_EN_PIN      GPIO_PIN_2
#define STEPPER_EN_PORT     GPIOA

typedef enum {
    STEPPER_DIR_FORWARD = 0,
    STEPPER_DIR_REVERSE = 1,
} Stepper_Dir_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;       /* e.g. TIM_CHANNEL_1 */
    float              omega_rad_s;   /* Current commanded speed */
    Stepper_Dir_t      direction;
    bool               running;
    volatile uint32_t  step_count;    /* Incremented in TIM update ISR */
} Stepper_Handle_t;

/* ---- Public API ---- */

/**
 * @brief  Initialise stepper driver. Motor will be DISABLED (ENN=HIGH) and
 *         timer stopped. Call before HAL_TIM_PWM_Start().
 */
void Stepper_Init(Stepper_Handle_t *hdrv,
                  TIM_HandleTypeDef *htim,
                  uint32_t channel);

/**
 * @brief  Enable motor (ENN=LOW) and start STEP generation at omega_rad_s.
 *         omega must be ≥ STEPPER_MIN_OMEGA_RAD_S.
 */
void Stepper_Start(Stepper_Handle_t *hdrv, float omega_rad_s, Stepper_Dir_t dir);

/**
 * @brief  Stop STEP generation and disable motor (coast).
 *         Call before any servo/hold operation.
 */
void Stepper_Stop(Stepper_Handle_t *hdrv);

/**
 * @brief  Update step rate atomically (ARR + CCR1 shadow registers).
 *         Call from speed controller at control loop rate.
 *         omega is clamped to [MIN, MAX] automatically.
 * @param  omega_rad_s  New spindle angular velocity (rad/s)
 */
void Stepper_SetSpeed(Stepper_Handle_t *hdrv, float omega_rad_s);

/**
 * @brief  Change direction. Only safe to call when stopped.
 */
void Stepper_SetDir(Stepper_Handle_t *hdrv, Stepper_Dir_t dir);

/**
 * @brief  Enable motor (ENN=LOW). Does not start pulses.
 */
void Stepper_Enable(Stepper_Handle_t *hdrv);

/**
 * @brief  Disable motor — coil currents off (ENN=HIGH). Motor can be back-driven.
 */
void Stepper_Disable(Stepper_Handle_t *hdrv);

/**
 * @brief  Call from TIM2 update ISR to count steps.
 *         In stm32f3xx_it.c: void TIM2_IRQHandler(void) { Stepper_ISR(&hstepper); }
 */
void Stepper_ISR(Stepper_Handle_t *hdrv);

/**
 * @brief  Convert omega (rad/s) to step rate (Hz). Exposed for diagnostics.
 */
static inline uint32_t Stepper_OmegaToHz(float omega_rad_s)
{
    /* step_hz = omega * usteps_per_rev / (2π) */
    return (uint32_t)(omega_rad_s * (float)STEPPER_USTEPS_PER_REV / 6.28318f);
}

#endif /* STEPPER_H */