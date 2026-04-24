#include "stepper.h"
#include <math.h>

/* -----------------------------------------------------------------------
 * Internal: compute ARR and CCR1 from omega
 * Returns false if omega is below minimum (timer should be stopped).
 * ----------------------------------------------------------------------- */
static bool omega_to_arr(float omega_rad_s, uint32_t *arr_out, uint32_t *ccr_out)
{
    if (omega_rad_s < STEPPER_MIN_OMEGA_RAD_S) return false;
    if (omega_rad_s > STEPPER_MAX_OMEGA_RAD_S) omega_rad_s = STEPPER_MAX_OMEGA_RAD_S;

    uint32_t step_hz = Stepper_OmegaToHz(omega_rad_s);
    if (step_hz == 0) return false;

    uint32_t arr = (STEPPER_TIMER_CLK_HZ / step_hz);
    if (arr < STEPPER_MIN_ARR + 1) arr = STEPPER_MIN_ARR + 1;
    if (arr > STEPPER_MAX_ARR + 1) arr = STEPPER_MAX_ARR + 1;
    arr -= 1;   /* ARR register value = period - 1 */

    *arr_out = arr;
    *ccr_out = arr / 2;   /* 50% duty — guarantees ≥ 12.5µs pulse at 40kHz */
    return true;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void Stepper_Init(Stepper_Handle_t *hdrv,
                  TIM_HandleTypeDef *htim,
                  uint32_t channel)
{
    hdrv->htim       = htim;
    hdrv->channel    = channel;
    hdrv->omega_rad_s = 0.0f;
    hdrv->direction  = STEPPER_DIR_FORWARD;
    hdrv->running    = false;
    hdrv->step_count = 0;

    /* Start with motor disabled, DIR=forward */
    HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STEPPER_EN_PORT,  STEPPER_EN_PIN,  GPIO_PIN_SET);  /* ENN=HIGH=disabled */

    /* Set a sane initial ARR so PWM doesn't glitch when first enabled */
    uint32_t arr, ccr;
    omega_to_arr(1.0f, &arr, &ccr);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}

void Stepper_Enable(Stepper_Handle_t *hdrv)
{
    HAL_GPIO_WritePin(STEPPER_EN_PORT, STEPPER_EN_PIN, GPIO_PIN_RESET);  /* ENN=LOW=enabled */
}

void Stepper_Disable(Stepper_Handle_t *hdrv)
{
    HAL_GPIO_WritePin(STEPPER_EN_PORT, STEPPER_EN_PIN, GPIO_PIN_SET);
}

void Stepper_SetDir(Stepper_Handle_t *hdrv, Stepper_Dir_t dir)
{
    hdrv->direction = dir;
    HAL_GPIO_WritePin(
        STEPPER_DIR_PORT, STEPPER_DIR_PIN,
        (dir == STEPPER_DIR_FORWARD) ? GPIO_PIN_RESET : GPIO_PIN_SET
    );
    /* DRV8434A requires DIR to be stable before STEP edge.
     * At our speeds this is always satisfied — no explicit delay needed. */
}

void Stepper_SetSpeed(Stepper_Handle_t *hdrv, float omega_rad_s)
{
    if (omega_rad_s < 0.0f) omega_rad_s = 0.0f;
    hdrv->omega_rad_s = omega_rad_s;

    uint32_t arr, ccr;
    bool valid = omega_to_arr(omega_rad_s, &arr, &ccr);

    if (!valid) {
        /* Speed too low — stop pulses but keep motor enabled (holding torque) */
        if (hdrv->running) {
            HAL_TIM_PWM_Stop(hdrv->htim, hdrv->channel);
            hdrv->running = false;
        }
        return;
    }

    if (!hdrv->running) {
        /* Need to (re)start timer — set ARR first, then start */
        __HAL_TIM_SET_AUTORELOAD(hdrv->htim, arr);
        __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->channel, ccr);
        HAL_TIM_PWM_Start_IT(hdrv->htim, hdrv->channel);
        hdrv->running = true;
    } else {
        /* Timer already running — update ARR and CCR atomically via shadow regs.
         * HAL_TIM_PWM_Start already enabled auto-reload preload, so the new
         * ARR takes effect at the next update event (no glitch). */
        __HAL_TIM_SET_AUTORELOAD(hdrv->htim, arr);
        __HAL_TIM_SET_COMPARE(hdrv->htim, hdrv->channel, ccr);
    }
}

void Stepper_Start(Stepper_Handle_t *hdrv, float omega_rad_s, Stepper_Dir_t dir)
{
    Stepper_SetDir(hdrv, dir);
    Stepper_Enable(hdrv);
    Stepper_SetSpeed(hdrv, omega_rad_s);
}

void Stepper_Stop(Stepper_Handle_t *hdrv)
{
    HAL_TIM_PWM_Stop(hdrv->htim, hdrv->channel);
    hdrv->running = false;
    /* Keep motor enabled (holding torque). Call Stepper_Disable() separately
     * if you want to coast. */
}

void Stepper_ISR(Stepper_Handle_t *hdrv)
{
    /* Called from TIM2_IRQHandler after HAL clears the flag.
     * One interrupt per step pulse (update event = one full ARR period = one step). */
    hdrv->step_count++;
}