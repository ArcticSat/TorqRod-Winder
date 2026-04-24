#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * DC Motor Driver — Pololu 4887 via Toshiba TB9051FTG H-Bridge
 * 
 * Timer:   TIM3 CH1 (PWM pin), 20kHz
 *          Prescaler=0 → 64MHz timer clock (APB1 timer clock)
 *          ARR=3199   → 20kHz PWM
 *
 * TB9051FTG Drive scheme:
 *   Forward:  IN1=LOW,   IN2=HIGH, PWM=Duty
 *   Reverse:  IN1=HIGH,  IN2=LOW,  PWM=Duty
 *   Brake:    IN1=HIGH,  IN2=HIGH, PWM=Don't care
 *   Coast:    IN1=LOW,   IN2=LOW,  PWM=Don't care
 * ----------------------------------------------------------------------- */

#define DCMOTOR_TIMER_ARR       3199U       /* ARR for 20kHz at 64MHz clock */
#define DCMOTOR_V_SUPPLY        12.0f       /* V — nominal supply */
#define DCMOTOR_V_MAX           12.0f       /* Clamp */
#define DCMOTOR_V_MIN          -12.0f

#define DCMOTOR_DEADBAND_V      0.05f

/* GPIO Pins for IN1 / IN2 (Standard outputs, NOT PWM) */
#define DCMOTOR_IN1_PIN         GPIO_PIN_7
#define DCMOTOR_IN1_PORT        GPIOA
#define DCMOTOR_IN2_PIN         GPIO_PIN_8
#define DCMOTOR_IN2_PORT        GPIOA

typedef enum {
    DCMOTOR_COAST = 0,
    DCMOTOR_FORWARD,
    DCMOTOR_REVERSE,
    DCMOTOR_BRAKE,
} DCMotor_Mode_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t ch_pwm;     /* TIM_CHANNEL_1 only */
    float    v_cmd;      /* Last commanded voltage */
    DCMotor_Mode_t mode;
} DCMotor_Handle_t;


/* ---- Public API ---- */

/**
 * @brief  Initialise driver and coast motor.
 */
void DCMotor_Init(DCMotor_Handle_t *hdrv, 
            TIM_HandleTypeDef *htim, 
            uint32_t ch_pwm);

/**
 * @brief  Set voltage command. Sign encodes direction.
 *         +V_cmd → pays out more wire (reduces tension).
 *         -V_cmd → brakes / back-drives spool (increases tension).
 *         Clamped to ±DCMOTOR_V_MAX. Dead-band applied.
 *         Call from tension controller at control loop rate.
 *
 * @param  v_cmd  Commanded voltage (V), signed.
 */
void DCMotor_SetVoltage(DCMotor_Handle_t *hdrv, float v_cmd);

/**
 * @brief  Hard brake — shorts both motor terminals through H-bridge.
 *         Use for e-stop or layer transition hold.
 */
void DCMotor_Brake(DCMotor_Handle_t *hdrv);

/**
 * @brief  Coast — both channels low. Motor free-wheels.
 *         Not normally used during winding.
 */
void DCMotor_Coast(DCMotor_Handle_t *hdrv);

/**
 * @brief  Return last commanded voltage.
 */
static inline float DCMotor_GetVcmd(const DCMotor_Handle_t *hdrv) {
    return hdrv->v_cmd;
}

#endif /* DC_MOTOR_H */