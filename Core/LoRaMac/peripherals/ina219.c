/*!
 * \file      ina219.c
 *
 * \brief     Minimal INA219 current/voltage monitor driver over I2C.
 */
#include "ina219.h"

/* Register map */
#define INA219_REG_CONFIG           0x00U
#define INA219_REG_SHUNT_VOLTAGE    0x01U
#define INA219_REG_BUS_VOLTAGE      0x02U
#define INA219_REG_POWER            0x03U
#define INA219_REG_CURRENT          0x04U
#define INA219_REG_CALIBRATION      0x05U

/* 32V range, /8 gain (320 mV shunt FSR), 12-bit bus/shunt ADC, continuous mode */
#define INA219_CONFIG_32V_2A        0x399FU
/* Calibration for a 0.1 ohm shunt with a 2 A max expected current:
 * Current_LSB = 0.1 mA/bit, Power_LSB = 2 mW/bit (see INA219 datasheet 8.5). */
#define INA219_CALIBRATION_32V_2A   4096U
#define INA219_CURRENT_LSB_UA       100

#define INA219_I2C_TIMEOUT_MS       50U

static I2C_HandleTypeDef *sHi2c;

static bool INA219_WriteReg(uint8_t reg, uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)(value & 0xFFU);

    return (HAL_I2C_Mem_Write(sHi2c, INA219_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                               data, sizeof(data), INA219_I2C_TIMEOUT_MS) == HAL_OK);
}

static bool INA219_ReadReg(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    if (HAL_I2C_Mem_Read(sHi2c, INA219_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                          data, sizeof(data), INA219_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return false;
    }

    *value = (uint16_t)((data[0] << 8) | data[1]);
    return true;
}

bool INA219_Init(I2C_HandleTypeDef *hi2c)
{
    sHi2c = hi2c;

    if (HAL_I2C_IsDeviceReady(sHi2c, INA219_I2C_ADDRESS, 2U, INA219_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return false;
    }

    if (!INA219_WriteReg(INA219_REG_CALIBRATION, INA219_CALIBRATION_32V_2A))
    {
        return false;
    }

    return INA219_WriteReg(INA219_REG_CONFIG, INA219_CONFIG_32V_2A);
}

bool INA219_ReadBusVoltage_mV(uint16_t *busVoltageMv)
{
    uint16_t raw;

    if (!INA219_ReadReg(INA219_REG_BUS_VOLTAGE, &raw))
    {
        return false;
    }

    /* Bits [15:3] hold the 4 mV/bit bus voltage, bits [2:0] are status flags. */
    *busVoltageMv = (uint16_t)((raw >> 3) * 4U);
    return true;
}

static bool INA219_ReadCurrentRaw(int16_t *raw)
{
    uint16_t value;

    /* Calibration register can reset to 0 after a bus brown-out; rewrite it
     * before every current read so the current register keeps producing
     * valid data. */
    if (!INA219_WriteReg(INA219_REG_CALIBRATION, INA219_CALIBRATION_32V_2A))
    {
        return false;
    }

    if (!INA219_ReadReg(INA219_REG_CURRENT, &value))
    {
        return false;
    }

    *raw = (int16_t)value;
    return true;
}

bool INA219_ReadCurrent_mA(int16_t *currentMa)
{
    int16_t raw;

    if (!INA219_ReadCurrentRaw(&raw))
    {
        return false;
    }

    *currentMa = (int16_t)(((int16_t)raw * INA219_CURRENT_LSB_UA) / 1000);
    return true;
}

bool INA219_ReadCurrent_x100uA(int16_t *current100uA)
{
    /* Register LSB is exactly the calibrated Current_LSB (100 uA per the
     * 32V/2A calibration above), so the raw value doubles as 0.1 mA units. */
    return INA219_ReadCurrentRaw(current100uA);
}
