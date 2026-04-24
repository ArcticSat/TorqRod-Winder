#include "dc_motor.h"
#include <math.h>

/* -----------------------------------------------------------------------
 * Internal helper: convert voltage to CCR value
 * ----------------------------------------------------------------------- */
static uint32_t voltage_to_ccr(float v_abs)
{
    if (v_abs < 0.0f) v_abs = 0.0f;
    if (v_abs > DCMOTOR_V_SUPPLY) v_abs = DCMOTOR_V_SUPPLY;
    return (uint32_t)((v_abs / DCMOTOR_V_SUPPLY) * (float)(DCMOTOR_TIMER_ARR + 1));
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void DCMotor_Init(DCMotor_Handle_t *hdrv, TIM_HandleTypeDef *htim, uint32_t ch_pwm)
{
    hdrv->htim   = htim;
    hdrv->ch_pwm = ch_pwm;
    hdrv->v_cmd  = 0.0f;
    hdrv->mode   = DCMOTOR_COAST;

    /* Start PWM channel at 0% duty, set IN1/IN2 LOW (Coast) */
    HAL_TIM_PWM_Start(htim, ch_pwm);
    __HAL_TIM_SET_COMPARE(htim, ch_pwm, 0);
    
    HAL_GPIO_WritePin(DCMOTOR_IN1_PORT, DCMOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DCMOTOR_IN2_PORT, DCMOTOR_IN2_PIN, GPIO_PIN_RESET);
}

void DCMotor_SetVoltage(DCMotor_Handle_t *hdrv, float v_cmd)
{
    if (v_cmd >  DCMOTOR_V_MAX) v_cmd =  DCMOTOR_V_MAX;
    if (v_cmd <  DCMOTOR_V_MIN) v_cmd =  DCMOTOR_V_MIN;
    hdrv->v_cmd = v_cmd;

    float v_abs = fabsf(v_cmd);

    /* Dead-band: brake at zero to hold spool position */
    if (v_abs < DCMOTOR_DEADBAND_V) {
        DCMotor_Brake(hdrv);
        return;
    }

    uint32_t ccr = voltage_to_ccr(v_abs);

    if (v_cmd > 0.0f) {
        /* Forward: IN1=LOW, IN2=HIGH, PWM=duty */
        hdrv->mode = DCMOTOR_FORWARD;
        HAL_GPIO_WritePin(DCMOTOR_IN1_PORT, DCMOTOR_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DCMOTOR_IN2_PORT, DCMOTOR_IN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_pwm, ccr);
    } else {
        /* Reverse: IN1=HIGH, IN2=LOW, PWM=duty */
        hdrv->mode = DCMOTOR_REVERSE;
        HAL_GPIO_WritePin(DCMOTOR_IN1_PORT, DCMOTOR_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DCMOTOR_IN2_PORT, DCMOTOR_IN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_pwm, ccr);
    }
}

void DCMotor_Brake(DCMotor_Handle_t *hdrv)
{
    /* TB9051FTG brake: both IN1 and IN2 = HIGH. PWM state is ignored by the IC. */
    hdrv->mode  = DCMOTOR_BRAKE;
    hdrv->v_cmd = 0.0f;
    
    __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_pwm, 0); /* Turn off PWM output */
    HAL_GPIO_WritePin(DCMOTOR_IN1_PORT, DCMOTOR_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DCMOTOR_IN2_PORT, DCMOTOR_IN2_PIN, GPIO_PIN_SET);
}

void DCMotor_Coast(DCMotor_Handle_t *hdrv)
{
    /* TB9051FTG coast: both IN1 and IN2 = LOW. PWM state is ignored. */
    hdrv->mode  = DCMOTOR_COAST;
    hdrv->v_cmd = 0.0f;
    
    __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_pwm, 0);
    HAL_GPIO_WritePin(DCMOTOR_IN1_PORT, DCMOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DCMOTOR_IN2_PORT, DCMOTOR_IN2_PIN, GPIO_PIN_RESET);
}