#ifndef DC_MOTOR_H
#define DC_MOTOR_H
#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
* DC Motor Driver — Pololu 2997 via Toshiba TB9051FTG H-Bridge
*
* Timer:   TIM3 CH1 + CH2 (PWM pins), 10kHz
*          Prescaler=0 → 32MHz timer clock (APB1 × 2)
*          ARR=3199   → 10kHz PWM
*
* TB9051FTG Drive scheme (Table 7.1-1 of datasheet):
*   Forward:  PWM1=Duty, PWM2=0,   EN=H, ENB=L → OUT1=H, OUT2=L
*   Reverse:  PWM1=0,    PWM2=Duty, EN=H, ENB=L → OUT1=L, OUT2=H
*   Brake:    PWM1=0,    PWM2=0,    EN=H, ENB=L → OUT1=L, OUT2=L (short brake)
*   Coast:    EN=L (or ENB=H) → outputs High-Z
*
*  EN is active HIGH, ENB is active LOW. Both must be set correctly.
* ----------------------------------------------------------------------- */
#define DCMOTOR_TIMER_ARR       3199U       // ARR for 10kHz at 32MHz timer clock 
#define DCMOTOR_V_SUPPLY        12.0f       // V — nominal supply
#define DCMOTOR_V_MAX           12.0f       // Clamp 
#define DCMOTOR_V_MIN          -12.0f
#define DCMOTOR_DEADBAND_V      0.15f       // Volts — prevent jitter on thin wire

// GPIO Pins 
#define DCMOTOR_EN_PORT         GPIOA
#define DCMOTOR_EN_PIN          GPIO_PIN_9   // EN: active HIGH 
// ENB: If GPIO-controlled, define below. Otherwise hardwire to GND and ignore. 
#define DCMOTOR_ENB_PORT        GPIOA
#define DCMOTOR_ENB_PIN         GPIO_PIN_8   /* ENB: active LOW (optional) */

/* PWM Channels on TIM3 */
#define DCMOTOR_PWM1_CH         TIM_CHANNEL_1  /* PA6 → TIM3_CH1 → PWM1 */
#define DCMOTOR_PWM2_CH         TIM_CHANNEL_2  /* PA7 → TIM3_CH2 → PWM2 (remapped from PA4) */

typedef enum {
    DCMOTOR_COAST = 0,
    DCMOTOR_FORWARD,
    DCMOTOR_REVERSE,
    DCMOTOR_BRAKE,
} DCMotor_Mode_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t ch_pwm1;   /* TIM_CHANNEL_1 */
    uint32_t ch_pwm2;   /* TIM_CHANNEL_2 */
    float    v_cmd;     /* Last commanded voltage */
    DCMotor_Mode_t mode;
} DCMotor_Handle_t;

/* ---- Public API ---- */
void DCMotor_Init(DCMotor_Handle_t *hdrv, TIM_HandleTypeDef *htim, 
                  uint32_t ch_pwm1, uint32_t ch_pwm2);

void DCMotor_SetVoltage(DCMotor_Handle_t *hdrv, float v_cmd);

void DCMotor_Brake(DCMotor_Handle_t *hdrv);

void DCMotor_Coast(DCMotor_Handle_t *hdrv);

static inline float DCMotor_GetVcmd(const DCMotor_Handle_t *hdrv) {
    return hdrv->v_cmd;
}

#endif /* DC_MOTOR_H */