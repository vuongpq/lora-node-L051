/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   Register-level ADC1 driver for LDR (PA1/IN1) and soil moisture
  *          (PA2/IN2) single-shot reads. The STM32L0xx_HAL_Driver package
  *          bundled with this project does not include stm32l0xx_hal_adc.c,
  *          so ADC1 is driven directly through its CMSIS registers instead
  *          of the HAL_ADC_* API.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "adc.h"

/* USER CODE BEGIN 0 */
#define ADC_CALIBRATION_TIMEOUT  100000U
#define ADC_READY_TIMEOUT        100000U
#define ADC_CONVERSION_TIMEOUT   100000U
/* USER CODE END 0 */

void MX_ADC_Init(void)
{
  /* USER CODE BEGIN ADC_Init 0 */
  __IO uint32_t timeout;
  /* USER CODE END ADC_Init 0 */

  __HAL_RCC_ADC1_CLK_ENABLE();

  /* ADC clock = PCLK2 / 2 (synchronous mode, no HSI16 dependency). */
  MODIFY_REG(ADC1->CFGR2, ADC_CFGR2_CKMODE, ADC_CFGR2_CKMODE_0);

  /* Self-calibration must run while ADEN = 0. */
  ADC1->CR |= ADC_CR_ADCAL;
  timeout = ADC_CALIBRATION_TIMEOUT;
  while ((ADC1->ISR & ADC_ISR_EOCAL) == 0U)
  {
    if (--timeout == 0U)
    {
      return;
    }
  }
  ADC1->ISR = ADC_ISR_EOCAL;

  ADC1->CR |= ADC_CR_ADEN;
  timeout = ADC_READY_TIMEOUT;
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U)
  {
    if (--timeout == 0U)
    {
      return;
    }
  }

  /* Longest sampling time available for high-impedance sensor dividers. */
  ADC1->SMPR = ADC_SMPR_SMP;
}

uint16_t AdcReadChannel(uint32_t channelSel)
{
  __IO uint32_t timeout;

  ADC1->CHSELR = channelSel;
  ADC1->CR |= ADC_CR_ADSTART;

  timeout = ADC_CONVERSION_TIMEOUT;
  while ((ADC1->ISR & ADC_ISR_EOC) == 0U)
  {
    if (--timeout == 0U)
    {
      return 0U;
    }
  }

  return (uint16_t)ADC1->DR;
}
