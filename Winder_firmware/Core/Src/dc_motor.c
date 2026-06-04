#include "dc_motor.h"
#include <math.h>

/* -----------------------------------------------------------------------
* Internal helper: convert voltage to CCR value
* HAL PWM duty = CCR / (ARR + 1)
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
void DCMotor_Init(DCMotor_Handle_t *hdrv, TIM_HandleTypeDef *htim, 
                  uint32_t ch_pwm1, uint32_t ch_pwm2)
{
    hdrv->htim    = htim;
    hdrv->ch_pwm1 = ch_pwm1;
    hdrv->ch_pwm2 = ch_pwm2;
    hdrv->v_cmd   = 0.0f;
    hdrv->mode    = DCMOTOR_COAST;

    /* 1. Enable TB9051FTG: EN=HIGH, ENB=LOW */
    HAL_GPIO_WritePin(DCMOTOR_EN_PORT, DCMOTOR_EN_PIN, GPIO_PIN_SET);
    /* If ENB is GPIO-controlled (not hardwired): */
    HAL_GPIO_WritePin(DCMOTOR_ENB_PORT, DCMOTOR_ENB_PIN, GPIO_PIN_RESET);

    /* 2. Start BOTH PWM channels at 0% duty */
    HAL_TIM_PWM_Start(htim, ch_pwm1);
    HAL_TIM_PWM_Start(htim, ch_pwm2);
    __HAL_TIM_SET_COMPARE(htim, ch_pwm1, 0);
    __HAL_TIM_SET_COMPARE(htim, ch_pwm2, 0);
}

void DCMotor_SetVoltage(DCMotor_Handle_t *hdrv, float v_cmd)
{
    if (v_cmd >  DCMOTOR_V_MAX) v_cmd =  DCMOTOR_V_MAX;
    if (v_cmd <  DCMOTOR_V_MIN) v_cmd =  DCMOTOR_V_MIN;
    hdrv->v_cmd = v_cmd;

    float v_abs = fabsf(v_cmd);

    /* Dead-band: Coast at zero to avoid snapping 36AWG wire */
    if (v_abs < DCMOTOR_DEADBAND_V) {
        DCMotor_Coast(hdrv);
        return;
    }

    uint32_t ccr = voltage_to_ccr(v_abs);

    if (v_cmd > 0.0f) {
        /* Forward: PWM1=Duty, PWM2=0 */
        hdrv->mode = DCMOTOR_FORWARD;
        __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_pwm1, ccr);
        __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_pwm2, 0);
    } else {
        /* Reverse: PWM1=0, PWM2=Duty */
        hdrv->mode = DCMOTOR_REVERSE;
        __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_pwm1, 0);
        __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_pwm2, ccr);
    }
}

void DCMotor_Brake(DCMotor_Handle_t *hdrv)
{
    /* TB9051FTG short brake: both PWM inputs LOW (or both HIGH) → OUT1=OUT2=LOW */
    hdrv->mode  = DCMOTOR_BRAKE;
    hdrv->v_cmd = 0.0f;
    __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_pwm1, 0);
    __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_pwm2, 0);
    /* Keep EN=HIGH, ENB=LOW so brake is active */
}

void DCMotor_Coast(DCMotor_Handle_t *hdrv)
{
    /* TB9051FTG coast: disable outputs by pulling EN LOW (or ENB HIGH) */
    hdrv->mode  = DCMOTOR_COAST;
    hdrv->v_cmd = 0.0f;
    __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_pwm1, 0);
    __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_pwm2, 0);
    /* Disable driver: EN=LOW (active HIGH pin) */
    HAL_GPIO_WritePin(DCMOTOR_EN_PORT, DCMOTOR_EN_PIN, GPIO_PIN_RESET);
}