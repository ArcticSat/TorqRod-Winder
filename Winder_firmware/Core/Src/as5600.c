#include "as5600.h"
#include <math.h>

/* -----------------------------------------------------------------------
 * Internal helpers
 * ----------------------------------------------------------------------- */

static AS5600_Status_t read_reg16(AS5600_Handle_t *hdev, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    HAL_StatusTypeDef r = HAL_I2C_Mem_Read(
        hdev->hi2c,
        AS5600_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf, 2,
        10 /* ms timeout */
    );
    if (r != HAL_OK) return AS5600_ERR_I2C;
    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return AS5600_OK;
}

static AS5600_Status_t read_reg8(AS5600_Handle_t *hdev, uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef r = HAL_I2C_Mem_Read(
        hdev->hi2c,
        AS5600_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        val, 1,
        10
    );
    return (r == HAL_OK) ? AS5600_OK : AS5600_ERR_I2C;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

AS5600_Status_t AS5600_Init(AS5600_Handle_t *hdev, I2C_HandleTypeDef *hi2c)
{
    hdev->hi2c        = hi2c;
    hdev->zero_offset = 0;
    hdev->theta_rad   = 0.0f;
    hdev->tension_N   = 0.0f;
    hdev->magnet_ok   = false;

    /* Verify device responds and magnet is present */
    AS5600_Status_t s = AS5600_CheckMagnet(hdev);
    return s;
}

AS5600_Status_t AS5600_CheckMagnet(AS5600_Handle_t *hdev)
{
    uint8_t status, agc;
    AS5600_Status_t s;

    s = read_reg8(hdev, AS5600_REG_STATUS, &status);
    if (s != AS5600_OK) return s;

    s = read_reg8(hdev, AS5600_REG_AGC, &agc);
    if (s != AS5600_OK) return s;

    hdev->agc = agc;

    if (!(status & AS5600_STATUS_MD)) {
        hdev->magnet_ok = false;
        return AS5600_ERR_NO_MAGNET;
    }
    if (status & AS5600_STATUS_MH) {
        hdev->magnet_ok = false;
        return AS5600_ERR_MAGNET_STRONG;
    }
    if (status & AS5600_STATUS_ML) {
        hdev->magnet_ok = false;
        return AS5600_ERR_MAGNET_WEAK;
    }

    hdev->magnet_ok = true;
    return AS5600_OK;
}

AS5600_Status_t AS5600_ReadRaw(AS5600_Handle_t *hdev, uint16_t *raw_out)
{
    uint16_t raw;
    AS5600_Status_t s = read_reg16(hdev, AS5600_REG_RAWANGLE_H, &raw);
    if (s != AS5600_OK) return s;
    *raw_out = raw & 0x0FFF;   /* 12-bit mask */
    return AS5600_OK;
}

AS5600_Status_t AS5600_CalibrateZero(AS5600_Handle_t *hdev)
{
    uint16_t raw;
    AS5600_Status_t s = AS5600_ReadRaw(hdev, &raw);
    if (s != AS5600_OK) return s;
    hdev->zero_offset = raw;
    hdev->theta_rad   = 0.0f;
    hdev->tension_N   = 0.0f;
    return AS5600_OK;
}

AS5600_Status_t AS5600_Update(AS5600_Handle_t *hdev)
{
    uint16_t raw;
    AS5600_Status_t s = AS5600_ReadRaw(hdev, &raw);
    if (s != AS5600_OK) return s;

    /* Apply zero offset with wraparound */
    int32_t delta = (int32_t)raw - (int32_t)hdev->zero_offset;
    if (delta < -2048) delta += 4096;
    if (delta >  2048) delta -= 4096;

    /* Convert counts → radians */
    hdev->theta_rad = (float)delta * AS5600_RAD_PER_COUNT;

    /* Clamp to physical range (arm can't go negative) */
    if (hdev->theta_rad < 0.0f) hdev->theta_rad = 0.0f;

    /* Tension from dancer arm spring model:
     *   T = K_sensor × theta
     *   K_sensor = K_spring / (2 * L_arm) = 0.0229 / 0.08 = 0.286 N/rad
     */
    hdev->tension_N = hdev->theta_rad * AS5600_K_SENSOR;

    return AS5600_OK;
}
