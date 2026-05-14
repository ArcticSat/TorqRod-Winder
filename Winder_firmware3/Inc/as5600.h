#ifndef AS5600_H
#define AS5600_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * AS5600 — 12-bit Magnetic Angle Encoder Driver
 * I2C address: 0x36 (7-bit, fixed)
 * Interface:   I2C, Fast Mode 400kHz
 *
 * Used for dancer arm angle sensing (CM7).
 * Angular operating range: 0–30° (tension sensing window)
 * Resolution: 0.087°/count → ~341 counts over 30°
 * ----------------------------------------------------------------------- */

#define AS5600_I2C_ADDR     (0x36 << 1)   /* HAL uses 8-bit shifted address */

/* Register addresses */
#define AS5600_REG_STATUS   0x0B
#define AS5600_REG_RAWANGLE_H 0x0C
#define AS5600_REG_RAWANGLE_L 0x0D
#define AS5600_REG_ANGLE_H  0x0E          /* Filtered + start/stop position applied */
#define AS5600_REG_ANGLE_L  0x0F
#define AS5600_REG_AGC      0x1A          /* Automatic gain control */
#define AS5600_REG_MAGNITUDE_H 0x1B
#define AS5600_REG_MAGNITUDE_L 0x1C

/* STATUS register bits */
#define AS5600_STATUS_MH    (1 << 3)      /* Magnet too strong */
#define AS5600_STATUS_ML    (1 << 4)      /* Magnet too weak   */
#define AS5600_STATUS_MD    (1 << 5)      /* Magnet detected   */

/* Physical constants for this application */
#define AS5600_COUNTS_FULL  4096U         /* 12-bit full scale */
#define AS5600_DEG_PER_COUNT (360.0f / 4096.0f)   /* 0.0879°/count */
#define AS5600_RAD_PER_COUNT (6.28318f / 4096.0f)

/* Dancer arm calibration:
 *   0° = arm horizontal → slack (T_min)
 *   5° = normal setpoint
 *  30° = hard stop (T_max, e-stop trigger)
 *
 *  K_sensor = K_spring / (2 * L_arm) = 0.0229 / (2 * 0.040) = 0.286 N/rad
 *  Tension = theta_rad * K_sensor
 */
#define AS5600_K_SENSOR     0.286f        /* N/rad */
#define AS5600_THETA_ZERO_COUNTS  0U      /* Raw count at 0° — calibrate at startup */

/* Return codes */
typedef enum {
    AS5600_OK      = 0,
    AS5600_ERR_I2C,
    AS5600_ERR_NO_MAGNET,
    AS5600_ERR_MAGNET_WEAK,
    AS5600_ERR_MAGNET_STRONG,
} AS5600_Status_t;

/* Driver handle */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t zero_offset;     /* Raw count subtracted to get arm angle */
    float    theta_rad;       /* Latest filtered angle (radians) */
    float    tension_N;       /* Latest computed tension (N) */
    uint8_t  agc;             /* AGC value (128=ideal, <50 or >200 = warn) */
    bool     magnet_ok;
} AS5600_Handle_t;

/* ---- Public API ---- */

/**
 * @brief  Initialise driver, verify magnet is present.
 * @param  hdev   Pointer to AS5600_Handle_t (user-allocated)
 * @param  hi2c   HAL I2C handle (I2C1, configured at 400kHz)
 * @return AS5600_OK on success
 */
AS5600_Status_t AS5600_Init(AS5600_Handle_t *hdev, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read raw 12-bit angle. Blocks until I2C complete (~50µs at 400kHz).
 * @param  hdev      Driver handle
 * @param  raw_out   Pointer to store raw count (0–4095)
 * @return AS5600_OK or error code
 */
AS5600_Status_t AS5600_ReadRaw(AS5600_Handle_t *hdev, uint16_t *raw_out);

/**
 * @brief  Read angle, apply zero offset, compute tension. Updates hdev fields.
 *         Call at control loop rate (e.g. 1kHz).
 * @param  hdev  Driver handle
 * @return AS5600_OK or error code
 */
AS5600_Status_t AS5600_Update(AS5600_Handle_t *hdev);

/**
 * @brief  Record current arm position as the zero reference.
 *         Call once at startup with arm at known 0° position.
 * @param  hdev  Driver handle
 */
AS5600_Status_t AS5600_CalibrateZero(AS5600_Handle_t *hdev);

/**
 * @brief  Read AGC and status, check magnet quality.
 * @param  hdev  Driver handle
 */
AS5600_Status_t AS5600_CheckMagnet(AS5600_Handle_t *hdev);

/**
 * @brief  Return latest tension reading in Newtons.
 */
static inline float AS5600_GetTension(const AS5600_Handle_t *hdev) {
    return hdev->tension_N;
}

/**
 * @brief  Return latest angle in radians.
 */
static inline float AS5600_GetAngle(const AS5600_Handle_t *hdev) {
    return hdev->theta_rad;
}

#endif /* AS5600_H */
