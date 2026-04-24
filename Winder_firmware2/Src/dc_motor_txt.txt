#include "dc_motor.h"
#include <math.h>

/* -----------------------------------------------------------------------
 * Internal helper: set CH1 and CH2 CCR values
 * ----------------------------------------------------------------------- */
static void set_duty(DCMotor_Handle_t *hdrv, uint32_t ccr1, uint32_t ccr2)
{
    __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_in1, ccr1);
    __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->ch_in2, ccr2);
}

static uint32_t voltage_to_ccr(float v_abs)
{
    /* Duty = |v| / V_supply, scaled to ARR */
    if (v_abs < 0.0f) v_abs = 0.0f;
    if (v_abs > DCMOTOR_V_SUPPLY) v_abs = DCMOTOR_V_SUPPLY;
    return (uint32_t)((v_abs / DCMOTOR_V_SUPPLY) * (float)(DCMOTOR_TIMER_ARR + 1));
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void DCMotor_Init(DCMotor_Handle_t *hdrv,
                  TIM_HandleTypeDef *htim,
                  uint32_t ch_in1,
                  uint32_t ch_in2)
{
    hdrv->htim   = htim;
    hdrv->ch_in1 = ch_in1;
    hdrv->ch_in2 = ch_in2;
    hdrv->v_cmd  = 0.0f;
    hdrv->mode   = DCMOTOR_COAST;

    /* Start both PWM channels at 0% duty (coast) */
    HAL_TIM_PWM_Start(htim, ch_in1);
    HAL_TIM_PWM_Start(htim, ch_in2);
    set_duty(hdrv, 0, 0);
}

void DCMotor_SetVoltage(DCMotor_Handle_t *hdrv, float v_cmd)
{
    /* Clamp */
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
        /* Forward: IN1=PWM, IN2=0
         * Positive voltage → motor drives spool forward → pays out wire
         * → tension drops. Sign convention matches tension controller:
         *   positive error (T < setpoint) → positive V_cmd → more wire out.
         */
        hdrv->mode = DCMOTOR_FORWARD;
        set_duty(hdrv, ccr, 0);
    } else {
        /* Reverse: IN1=0, IN2=PWM — brakes/back-drives spool */
        hdrv->mode = DCMOTOR_REVERSE;
        set_duty(hdrv, 0, ccr);
    }
}

void DCMotor_Brake(DCMotor_Handle_t *hdrv)
{
    /* TB9051FTG brake: both IN1 and IN2 = HIGH (100% duty)
     * The H-bridge connects both motor terminals to ground — current decays
     * through the winding resistance (slow decay / fast braking). */
    hdrv->mode  = DCMOTOR_BRAKE;
    hdrv->v_cmd = 0.0f;
    set_duty(hdrv, DCMOTOR_TIMER_ARR + 1, DCMOTOR_TIMER_ARR + 1);
}

void DCMotor_Coast(DCMotor_Handle_t *hdrv)
{
    hdrv->mode  = DCMOTOR_COAST;
    hdrv->v_cmd = 0.0f;
    set_duty(hdrv, 0, 0);
}