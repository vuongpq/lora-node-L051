/*!
 * \file      ina219.h
 *
 * \brief     Minimal INA219 current/voltage monitor driver over I2C.
 */
#ifndef __INA219_H__
#define __INA219_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

/*! Default 7-bit I2C address when A0/A1 are tied to GND */
#define INA219_I2C_ADDRESS          (0x40U << 1)

/*!
 * \brief   Configures the calibration register for a 0.1 ohm shunt with a
 *          32 V / 2 A measurement range (matches common INA219 breakout
 *          boards). Must be called once before taking readings.
 *
 * \param   [IN] hi2c - I2C bus handle the sensor is attached to.
 *
 * \retval  true if the sensor answered and was configured successfully.
 */
bool INA219_Init(I2C_HandleTypeDef *hi2c);

/*!
 * \brief   Reads the bus voltage.
 *
 * \param   [OUT] busVoltageMv - Bus voltage in millivolts.
 *
 * \retval  true on success.
 */
bool INA219_ReadBusVoltage_mV(uint16_t *busVoltageMv);

/*!
 * \brief   Reads the shunt current (signed, can be negative if the current
 *          direction is reversed across the shunt resistor).
 *
 * \param   [OUT] currentMa - Current in milliamps.
 *
 * \retval  true on success.
 */
bool INA219_ReadCurrent_mA(int16_t *currentMa);

/*!
 * \brief   Reads the shunt current at the calibrated LSB resolution
 *          (100 uA/bit), for callers that need finer-than-1-mA precision
 *          (e.g. reporting sub-mA idle current in a single payload byte).
 *
 * \param   [OUT] current100uA - Current in units of 100 uA (0.1 mA).
 *
 * \retval  true on success.
 */
bool INA219_ReadCurrent_x100uA(int16_t *current100uA);

#ifdef __cplusplus
}
#endif

#endif /* __INA219_H__ */
