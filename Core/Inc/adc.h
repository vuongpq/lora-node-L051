/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

void MX_ADC_Init(void);

/* USER CODE BEGIN Prototypes */
/*!
 * \brief   Reads a single ADC1 channel (register-level driver; no HAL_ADC
 *          module is vendored for this target).
 *
 * \param   [IN] channelSel - One of the ADC_CHSELR_CHSELx bit masks.
 *
 * \retval  12-bit right-aligned conversion result, or 0 on timeout.
 */
uint16_t AdcReadChannel(uint32_t channelSel);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
