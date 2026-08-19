/*!
 * \file      dht22.c
 *
 * \brief     Bit-banged DHT22 (AM2302) temperature/humidity driver.
 *
 *            Cortex-M0+ has no DWT cycle counter, so TIM2 is free-run at
 *            1 MHz to time the single-wire protocol instead.
 */
#include "dht22.h"
#include "main.h"

/* Some AM2302/DHT22 clones need a longer, unambiguous start pulse and a bit
 * more handshake margin than the datasheet minimum to reliably wake up. */
#define DHT22_START_LOW_MS            10U
#define DHT22_HANDSHAKE_TIMEOUT_US    1000U
#define DHT22_BIT_LEAD_TIMEOUT_US     150U
#define DHT22_BIT_DATA_TIMEOUT_US     150U
#define DHT22_BIT_ONE_THRESHOLD_US    40U

volatile uint8_t gDht22LastStage = 0;

static void Dht22SetOutputLow(void)
{
  GPIO_InitTypeDef gpio = {0};

  HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);
  gpio.Pin = DHT22_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DHT22_GPIO_Port, &gpio);
}

static void Dht22SetInput(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = DHT22_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT22_GPIO_Port, &gpio);
}

/* Blocks until the line reaches the requested level.
 * Returns the elapsed microseconds, or -1 on timeout.
 * Bounded by both TIM2 and SysTick so a stalled/unstarted TIM2 (or a
 * disconnected sensor) can never hang the caller forever. */
static int32_t Dht22WaitForLevel(GPIO_PinState level, uint16_t timeoutUs)
{
  uint16_t start = (uint16_t)TIM2->CNT;
  uint32_t tickStart = HAL_GetTick();
  uint16_t elapsed;

  while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) != level)
  {
    elapsed = (uint16_t)((uint16_t)TIM2->CNT - start);
    if (elapsed > timeoutUs)
    {
      return -1;
    }
    if ((HAL_GetTick() - tickStart) > 5U)
    {
      return -1;
    }
  }

  return (int32_t)((uint16_t)((uint16_t)TIM2->CNT - start));
}

/* rtc-board.c also drives TIM2 (as the LoRaMac alarm timer) and leaves it
 * stopped (CR1.CEN cleared) once the stack arms its first software timer,
 * which happens well before the first sensor read. Re-arming the free-run
 * config here - not just once in Dht22Init() - keeps this driver immune to
 * whatever state RtcSetAlarm()/Tim2Stop() left the peripheral in. */
static void Dht22TimerStart(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM2->CR1 = 0U;
  TIM2->PSC = (uint16_t)((SystemCoreClock / 1000000U) - 1U);
  TIM2->ARR = 0xFFFFU;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0U;
  TIM2->CR1 = TIM_CR1_CEN;
}

void Dht22Init(void)
{
  Dht22TimerStart();
  Dht22SetInput();
}

bool Dht22Read(int16_t *temperatureCx10, uint16_t *humidityPctx10)
{
  uint8_t data[5] = {0};
  uint8_t byteIdx;
  uint8_t bitIdx;
  uint16_t humidityRaw;
  uint16_t temperatureRaw;

  /* Reclaim TIM2 in case rtc-board.c's RtcSetAlarm() stopped it since the
   * last read (see Dht22TimerStart() comment). */
  Dht22TimerStart();

  /* Start signal: pull the bus low for >=1 ms, then release it. */
  Dht22SetOutputLow();
  HAL_Delay(DHT22_START_LOW_MS);
  Dht22SetInput();

  /* Bus release (pulled high) -> sensor ACK low -> sensor ACK high. */
  if (Dht22WaitForLevel(GPIO_PIN_SET, DHT22_HANDSHAKE_TIMEOUT_US) < 0)
  {
    gDht22LastStage = 1U;
    return false;
  }
  if (Dht22WaitForLevel(GPIO_PIN_RESET, DHT22_HANDSHAKE_TIMEOUT_US) < 0)
  {
    gDht22LastStage = 2U;
    return false;
  }
  if (Dht22WaitForLevel(GPIO_PIN_SET, DHT22_HANDSHAKE_TIMEOUT_US) < 0)
  {
    gDht22LastStage = 3U;
    return false;
  }
  /* Falling edge here doubles as the first bit's 50us lead pulse. */
  if (Dht22WaitForLevel(GPIO_PIN_RESET, DHT22_HANDSHAKE_TIMEOUT_US) < 0)
  {
    gDht22LastStage = 4U;
    return false;
  }

  for (byteIdx = 0U; byteIdx < 5U; byteIdx++)
  {
    for (bitIdx = 0U; bitIdx < 8U; bitIdx++)
    {
      int32_t highUs;
      uint8_t bitNumber = (uint8_t)((byteIdx * 8U) + bitIdx);

      if (Dht22WaitForLevel(GPIO_PIN_SET, DHT22_BIT_LEAD_TIMEOUT_US) < 0)
      {
        gDht22LastStage = (uint8_t)(10U + bitNumber);
        return false;
      }

      highUs = Dht22WaitForLevel(GPIO_PIN_RESET, DHT22_BIT_DATA_TIMEOUT_US);
      if (highUs < 0)
      {
        gDht22LastStage = (uint8_t)(50U + bitNumber);
        return false;
      }

      data[byteIdx] = (uint8_t)(data[byteIdx] << 1);
      if (highUs > DHT22_BIT_ONE_THRESHOLD_US)
      {
        data[byteIdx] |= 1U;
      }
    }
  }

  if (((uint8_t)(data[0] + data[1] + data[2] + data[3])) != data[4])
  {
    gDht22LastStage = 90U;
    return false;
  }

  humidityRaw = (uint16_t)((data[0] << 8) | data[1]);
  temperatureRaw = (uint16_t)(((data[2] & 0x7FU) << 8) | data[3]);

  *humidityPctx10 = humidityRaw;
  *temperatureCx10 = (data[2] & 0x80U) ? -(int16_t)temperatureRaw : (int16_t)temperatureRaw;

  gDht22LastStage = 0U;
  return true;
}
