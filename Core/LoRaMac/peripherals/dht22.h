/*!
 * \file      dht22.h
 *
 * \brief     Bit-banged DHT22 (AM2302) temperature/humidity driver.
 */
#ifndef __DHT22_H__
#define __DHT22_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*!
 * Diagnostic code set by the last Dht22Read() call:
 *   0        = success
 *   1-4      = failed handshake step 1-4 (release-high, ack-low, ack-high, first-bit-lead)
 *   10..49   = failed waiting for a bit's lead pulse (10 + bit index 0-39)
 *   50..89   = failed waiting for a bit's data pulse (50 + bit index 0-39)
 *   90       = checksum mismatch
 */
extern volatile uint8_t gDht22LastStage;

/*!
 * \brief   Starts the free-running microsecond timer used to bit-bang the
 *          single-wire protocol. Must be called once, after the system
 *          clock is configured.
 */
void Dht22Init(void);

/*!
 * \brief   Performs a full DHT22 read cycle.
 *
 * \param   [OUT] temperatureCx10 - Temperature in tenths of a degree Celsius.
 * \param   [OUT] humidityPctx10  - Relative humidity in tenths of a percent.
 *
 * \retval  true if the handshake, all 40 bits and the checksum were valid.
 */
bool Dht22Read(int16_t *temperatureCx10, uint16_t *humidityPctx10);

#ifdef __cplusplus
}
#endif

#endif /* __DHT22_H__ */
