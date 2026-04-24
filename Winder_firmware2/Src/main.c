/* =========================================================================
 * main.c — Inductor Winder Application
 *
 * Bring-up sequence and main control loop skeleton.
 *
 * Hardware:   STM32F303K8
 * Peripherals: I2C1 (AS5600), TIM2/CH1 (stepper), TIM3/CH1+CH2 (DC motor)
 * ========================================================================= */

#include "main.h"           /* CubeMX generated — contains handle declarations */
#include "as5600.h"
#include "stepper.h"
#include "dc_motor.h"
#include <stdio.h>          /* For printf via semihosting or ITM */

/* -------------------------------------------------------------------------
 * Extern HAL handles (defined in CubeMX main.c)
 * ------------------------------------------------------------------------- */
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim2;   /* Stepper STEP pulses */
extern TIM_HandleTypeDef  htim3;   /* DC motor PWM        */

/* -------------------------------------------------------------------------
 * Driver handles
 * ------------------------------------------------------------------------- */
AS5600_Handle_t  henc;    /* Dancer arm encoder */
Stepper_Handle_t hstep;   /* Spindle stepper    */
DCMotor_Handle_t hdcmot;  /* Spool DC motor     */

/* -------------------------------------------------------------------------
 * Winder state
 * ------------------------------------------------------------------------- */
typedef enum {
    STATE_INIT = 0,
    STATE_IDLE,
    STATE_PRETENSION,   /* Ramp tension to T_pretension before spinning */
    STATE_WINDING,
    STATE_ESTOP,
} WinderState_t;

static WinderState_t winder_state = STATE_INIT;

/* Control parameters (from simulation tuning — starting points for hardware) */
static const float T_SETPOINT   = 0.025f;  /* N */
static const float T_MIN        = 0.005f;  /* N — slack e-stop threshold */
static const float T_MAX        = 0.080f;  /* N — overtension e-stop threshold */
static const float T_PRETENSION = 0.010f;  /* N */
static const float KP_TENSION   = 90.0f;   /* V/N */
static const float KI_TENSION   = 170.0f;  /* V/(N·s) */
static const float OMEGA_TARGET = 2.0f;    /* rad/s spindle — tune to desired RPM */

/* PI state */
static float tension_integral = 0.0f;
static const float ANTI_WINDUP = 12.0f;    /* V */

/* Control loop period */
static const float DT = 0.001f;   /* 1kHz — driven by SysTick / TIM interrupt */
static volatile bool control_tick = false;

/* -------------------------------------------------------------------------
 * E-stop
 * ------------------------------------------------------------------------- */
static void estop(const char *reason)
{
    Stepper_Stop(&hstep);
    Stepper_Disable(&hstep);
    DCMotor_Brake(&hdcmot);
    winder_state = STATE_ESTOP;
    (void)reason;
    /* TODO: signal fault to UI / LED */
}

/* -------------------------------------------------------------------------
 * Tension PI controller
 * Returns V_cmd for spool motor.
 * ------------------------------------------------------------------------- */
static float tension_pi(float t_measured)
{
    float error = T_SETPOINT - t_measured;

    tension_integral += error * DT;
    /* Anti-windup clamp */
    float max_i = ANTI_WINDUP / KI_TENSION;
    if (tension_integral >  max_i) tension_integral =  max_i;
    if (tension_integral < -max_i) tension_integral = -max_i;

    float v_cmd = KP_TENSION * error + KI_TENSION * tension_integral;

    if (v_cmd >  ANTI_WINDUP) v_cmd =  ANTI_WINDUP;
    if (v_cmd < -ANTI_WINDUP) v_cmd = -ANTI_WINDUP;

    return v_cmd;
}

/* -------------------------------------------------------------------------
 * Winder_Init — call once at startup
 * ------------------------------------------------------------------------- */
void Winder_Init(void)
{
    /* 1. AS5600 — must be first; we need tension before enabling motion */
    AS5600_Status_t enc_status = AS5600_Init(&henc, &hi2c1);
    if (enc_status != AS5600_OK) {
        /* No magnet or I2C fault — cannot safely wind */
        estop("AS5600 init failed");
        return;
    }

    /* 2. Calibrate zero — arm must be at rest / slack position now */
    AS5600_CalibrateZero(&henc);

    /* 3. Stepper */
    Stepper_Init(&hstep, &htim2, TIM_CHANNEL_1);

    /* 4. DC motor */
    DCMotor_Init(&hdcmot, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2);

    winder_state = STATE_IDLE;
}

/* -------------------------------------------------------------------------
 * Winder_Start — call when user commands a winding run
 * ------------------------------------------------------------------------- */
void Winder_Start(void)
{
    if (winder_state != STATE_IDLE) return;
    tension_integral = 0.0f;
    winder_state = STATE_PRETENSION;
}

/* -------------------------------------------------------------------------
 * Winder_ControlLoop — call at 1kHz (from SysTick or timer ISR flag)
 *
 * In your stm32f3xx_it.c SysTick_Handler, set control_tick = true.
 * Call Winder_ControlLoop() from main loop when control_tick is true.
 * ------------------------------------------------------------------------- */
void Winder_ControlLoop(void)
{
    /* 1. Read sensor */
    AS5600_Status_t enc_status = AS5600_Update(&henc);
    if (enc_status != AS5600_OK) {
        estop("AS5600 read error");
        return;
    }
    float t_measured = AS5600_GetTension(&henc);

    /* 2. Safety limits */
    if (t_measured > T_MAX) { estop("Overtension"); return; }
    if (winder_state == STATE_WINDING && t_measured < T_MIN) {
        estop("Slack / wire break");
        return;
    }

    switch (winder_state) {

    case STATE_PRETENSION: {
        /* Hold spindle still, ramp spool until T_pretension is reached */
        float v_cmd = KP_TENSION * (T_PRETENSION - t_measured);
        if (v_cmd >  6.0f) v_cmd =  6.0f;   /* Gentle during pretension */
        if (v_cmd < -6.0f) v_cmd = -6.0f;
        DCMotor_SetVoltage(&hdcmot, v_cmd);

        if (t_measured >= T_PRETENSION * 0.9f) {
            /* Close enough — start spindle */
            Stepper_Start(&hstep, OMEGA_TARGET, STEPPER_DIR_FORWARD);
            winder_state = STATE_WINDING;
        }
        break;
    }

    case STATE_WINDING: {
        /* Full PI tension controller */
        float v_cmd = tension_pi(t_measured);
        DCMotor_SetVoltage(&hdcmot, v_cmd);

        /* Speed controller: feedforward only for now.
         * TODO: add closed-loop correction using cam position or encoder. */
        Stepper_SetSpeed(&hstep, OMEGA_TARGET);
        break;
    }

    case STATE_IDLE:
    case STATE_ESTOP:
    case STATE_INIT:
    default:
        break;
    }
}

/* -------------------------------------------------------------------------
 * ISR hooks — put these in stm32f3xx_it.c
 * ------------------------------------------------------------------------- */

/* TIM2 Update (stepper step count) */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)) {
        Stepper_ISR(&hstep);
    }
}

/* -------------------------------------------------------------------------
 * main() integration — add to your CubeMX main.c:
 *
 *   Winder_Init();
 *
 *   while (1) {
 *       if (control_tick) {
 *           control_tick = false;
 *           Winder_ControlLoop();
 *       }
 *       // other tasks (UART, button polling, etc.)
 *   }
 * ------------------------------------------------------------------------- */