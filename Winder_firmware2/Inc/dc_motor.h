#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * DC Motor Driver — Pololu 4887 brushed gearmotor via Toshiba TB9051FTG H-bridge
 *
 * Timer:   TIM3 CH1 (IN1) + CH2 (IN2), both PWM, 10kHz
 *          Prescaler=0 → 32MHz timer clock (APB1×2)
 *          ARR=3199   → 10kHz PWM
 *
 * Drive scheme: SLOW DECAY (coast-free) — preferred for low-speed tension control.
 *   Forward:  IN1=PWM(duty),  IN2=0
 *   Reverse:  IN1=0,          IN2=PWM(duty)
 *   Brake:    IN1=1,          IN2=1    (both HIGH → H-bridge brake)
 *   Coast:    IN1=0,          IN2=0
 *
 * Voltage command interface:
 *   Tension controller outputs V_cmd in range [-12, +12] V.
 *   This is converted to duty cycle 0–100% on the appropriate channel.
 *   Supply voltage is nominally 12V.
 *
 * Motor (Pololu 4887):
 *   Kt = Ke = 0.0198 Nm/A
 *   R  = 13.3Ω   (measure and confirm on arrival)
 *   Gear ratio 98.78:1
 *   Max output: ~56.7 RPM, stall current 0.9A
 *
 * TB9051FTG limits: 2.7A continuous, 5A peak — well within Pololu stall spec.
 * ----------------------------------------------------------------------- */

#define DCMOTOR_TIMER_ARR       3199U       /* ARR for 10kHz at 32MHz clock */
#define DCMOTOR_V_SUPPLY        12.0f       /* V — nominal supply */
#define DCMOTOR_V_MAX           12.0f       /* Clamp */
#define DCMOTOR_V_MIN          -12.0f

/* Dead-band: below this |V_cmd| just brake (avoids chattering at rest) */
#define DCMOTOR_DEADBAND_V      0.05f

typedef enum {
    DCMOTOR_COAST = 0,
    DCMOTOR_FORWARD,
    DCMOTOR_REVERSE,
    DCMOTOR_BRAKE,
} DCMotor_Mode_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t ch_in1;    /* TIM_CHANNEL_1 */
    uint32_t ch_in2;    /* TIM_CHANNEL_2 */
    float    v_cmd;     /* Last commanded voltage */
    DCMotor_Mode_t mode;
} DCMotor_Handle_t;

/* ---- Public API ---- */

/**
 * @brief  Initialise driver and coast motor.
 */
void DCMotor_Init(DCMotor_Handle_t *hdrv,
                  TIM_HandleTypeDef *htim,
                  uint32_t ch_in1,
                  uint32_t ch_in2);

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