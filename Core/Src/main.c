/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "main.h"
#include "rtc.h"
#include "spi.h"
#include "i2c.h"
#include "adc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board.h"
#include "rtc-board.h"
#include "LoRaMac.h"
#include "secure-element.h"
#include "timer.h"
#include "ina219.h"
#include "dht22.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LORAWAN_ACTIVE_REGION      LORAMAC_REGION_AS923
#define APP_TX_PORT                2U
#define APP_PAYLOAD_SIZE           14U
#define SENSOR_POWERON_DELAY_MS    2000U
#define APP_TX_PERIOD_MS           30000U
#define APP_JOIN_RETRY_MS          10000U
#define APP_JOIN_CONFIRM_WAIT_MS   15000U
#define APP_RADIO_PROBE_RETRIES    10U
#define APP_RADIO_PROBE_DELAY_MS   20U
#define APP_JOIN_DATARATE          DR_2
#define APP_TX_DATARATE            DR_4
#define JOIN_ACCEPT_DELAY1_MS      5000UL
#define JOIN_ACCEPT_DELAY2_MS      6000UL
#define RX_TIMING_ERROR_MS         40UL
#define RX_MIN_SYMBOLS             12U
#define RX2_FREQUENCY_AS923        923200000UL
#define RX2_DR_AS923               DR_2
/* Adjust to the real pack's cut-off/full-charge voltage. */
#define APP_BATTERY_VOLTAGE_MIN_MV 3300U
#define APP_BATTERY_VOLTAGE_MAX_MV 4200U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static bool AppJoined = false;
static bool AppJoinRequested = false;
static uint32_t AppNextJoinTime = 0;
static uint32_t AppNextTxTime = 0;
static uint32_t AppFrameCounter = 0;
static uint32_t AppJoinRequestTick = 0;
static uint32_t AppJoinAttemptCount = 0;
static uint8_t AppTxBuffer[APP_PAYLOAD_SIZE];
static bool AppIna219Present = false;
static bool AppSensorPowerOn = false;
static uint32_t AppSensorPowerOnTick = 0;
/* Sampled once per cycle, right before the sensor cluster is powered on for
 * the next read - i.e. while it's still off and the radio is asleep. Units
 * of 0.1 mA. Note this still reflects the MCU in full RUN mode (no STOP
 * mode is implemented), not a true low-power idle current. */
static uint8_t AppIdleCurrentX100uA = 0;

typedef struct
{
  uint32_t magic;
  uint32_t mainLoopCount;
  uint32_t isJoined;
  uint32_t joinReqCount;
  uint32_t uplinkReqCount;
  uint32_t uplinkConfirmCount;
  uint32_t lastMcpsRequestStatus;
  uint32_t lastMcpsStatus;
  uint32_t macBusyCount;
  uint32_t dio0IrqCount;
  uint32_t dio1IrqCount;
  uint32_t frameCounter;
  uint32_t dhtOkCount;
  uint32_t dhtFailCount;
  uint32_t dhtLastStage;
  uint32_t lightRaw;
  uint32_t soilRaw;
} AppDiagSnapshot_t;

#define APP_DIAG_MAGIC 0x44494147U /* "DIAG" */
__attribute__((section(".noinit"))) volatile AppDiagSnapshot_t gDiagSnapshot;
extern volatile uint32_t gDbgDio0IrqCount;
extern volatile uint32_t gDbgDio1IrqCount;
extern volatile uint8_t gDht22LastStage;

static void AppUpdateDiagSnapshot(void)
{
  gDiagSnapshot.magic = APP_DIAG_MAGIC;
  gDiagSnapshot.isJoined = AppJoined ? 1U : 0U;
  gDiagSnapshot.joinReqCount = AppJoinAttemptCount;
  gDiagSnapshot.dio0IrqCount = gDbgDio0IrqCount;
  gDiagSnapshot.dio1IrqCount = gDbgDio1IrqCount;
  gDiagSnapshot.frameCounter = AppFrameCounter;
}

static LoRaMacPrimitives_t AppPrimitives;
static LoRaMacCallback_t AppCallbacks;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t AppComputeBatteryLevel(uint16_t busVoltageMv);
static uint8_t AppGetBatteryLevel(void);
static float AppGetTemperatureLevel(void);
static void AppNvmDataChange(uint16_t notifyFlags);
static void AppMacProcessNotify(void);
static void AppMacMcpsConfirm(McpsConfirm_t *mcpsConfirm);
static void AppMacMcpsIndication(McpsIndication_t *mcpsIndication);
static void AppMacMlmeConfirm(MlmeConfirm_t *mlmeConfirm);
static void AppMacMlmeIndication(MlmeIndication_t *mlmeIndication);
static bool AppConfigureMac(void);
static void AppRequestJoin(void);
static void AppSendUplink(void);
static void AppProcess(void);
static void AppInit(void);
static bool AppRadioSanityCheck(void);
extern void SX1276IoInit(void);
extern void SX1276Reset(void);
extern uint8_t SX1276Read(uint32_t addr);
extern void GpioMcuIrqHandler(uint16_t gpioPin);
/* USER CODE END PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  GpioMcuIrqHandler(GPIO_Pin);
}

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t AppComputeBatteryLevel(uint16_t busVoltageMv)
{
  uint32_t level;

  /* LoRaWAN convention: 0 = external power, 1..254 = battery level,
   * 255 = unable to measure. */
  if (busVoltageMv == 0U)
  {
    return 255U;
  }
  if (busVoltageMv <= APP_BATTERY_VOLTAGE_MIN_MV)
  {
    return 1U;
  }
  if (busVoltageMv >= APP_BATTERY_VOLTAGE_MAX_MV)
  {
    return 254U;
  }

  level = ((uint32_t)(busVoltageMv - APP_BATTERY_VOLTAGE_MIN_MV) * 253U) /
          (APP_BATTERY_VOLTAGE_MAX_MV - APP_BATTERY_VOLTAGE_MIN_MV);
  return (uint8_t)(1U + level);
}

/* Callback for LoRaMac's own DevStatusAns handling (independent of uplinks). */
static uint8_t AppGetBatteryLevel(void)
{
  uint16_t busVoltageMv = 0U;

  if (AppIna219Present)
  {
    (void)INA219_ReadBusVoltage_mV(&busVoltageMv);
  }

  return AppComputeBatteryLevel(busVoltageMv);
}

static float AppGetTemperatureLevel(void)
{
  return 25.0f;
}

static void AppNvmDataChange(uint16_t notifyFlags)
{
  (void)notifyFlags;
}

static void AppMacProcessNotify(void)
{
}

static void AppMacMcpsConfirm(McpsConfirm_t *mcpsConfirm)
{
  if (mcpsConfirm == NULL)
  {
    return;
  }

  gDiagSnapshot.uplinkConfirmCount++;
  gDiagSnapshot.lastMcpsStatus = (uint32_t)mcpsConfirm->Status;

  if (mcpsConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK)
  {
    AppNextTxTime = HAL_GetTick() + APP_TX_PERIOD_MS;
  }
}

static void AppMacMcpsIndication(McpsIndication_t *mcpsIndication)
{
  (void)mcpsIndication;
}

static void AppMacMlmeConfirm(MlmeConfirm_t *mlmeConfirm)
{
  if (mlmeConfirm == NULL)
  {
    return;
  }

  if (mlmeConfirm->MlmeRequest == MLME_JOIN)
  {
    AppJoinRequested = false;

    if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
    {
      MibRequestConfirm_t mibReq = {0};

      /* ADR ignores McpsRequest.Datarate once enabled, so the ADR-tracked
       * datarate must be raised here or it stays at the join datarate,
       * which is too small for our payload (LORAMAC_STATUS_LENGTH_ERROR). */
      mibReq.Type = MIB_CHANNELS_DATARATE;
      mibReq.Param.ChannelsDatarate = APP_TX_DATARATE;
      (void)LoRaMacMibSetRequestConfirm(&mibReq);

      AppJoined = true;
      AppNextTxTime = HAL_GetTick() + 5000U;
    }
    else
    {
      AppJoined = false;
      AppNextJoinTime = HAL_GetTick() + APP_JOIN_RETRY_MS;
    }
  }
}

static void AppMacMlmeIndication(MlmeIndication_t *mlmeIndication)
{
  (void)mlmeIndication;
}

static bool AppConfigureMac(void)
{
  MibRequestConfirm_t mibReq = {0};
  LoRaMacStatus_t st;
  mibReq.Type = MIB_DEVICE_CLASS;
  mibReq.Param.Class = CLASS_A;
  st = LoRaMacMibSetRequestConfirm(&mibReq);
  if (st != LORAMAC_STATUS_OK)
  {
    return false;
  }

  mibReq.Type = MIB_DEV_EUI;
  mibReq.Param.DevEui = SecureElementGetDevEui();
  st = LoRaMacMibSetRequestConfirm(&mibReq);
  if (st != LORAMAC_STATUS_OK)
  {
    return false;
  }

  mibReq.Type = MIB_JOIN_EUI;
  mibReq.Param.JoinEui = SecureElementGetJoinEui();
  st = LoRaMacMibSetRequestConfirm(&mibReq);
  if (st != LORAMAC_STATUS_OK)
  {
    return false;
  }

  mibReq.Type = MIB_ADR;
  mibReq.Param.AdrEnable = true;
  st = LoRaMacMibSetRequestConfirm(&mibReq);
  if (st != LORAMAC_STATUS_OK)
  {
    return false;
  }

  mibReq.Type = MIB_JOIN_ACCEPT_DELAY_1;
  mibReq.Param.JoinAcceptDelay1 = JOIN_ACCEPT_DELAY1_MS;
  st = LoRaMacMibSetRequestConfirm(&mibReq);
  if (st != LORAMAC_STATUS_OK)
  {
    return false;
  }

  mibReq.Type = MIB_JOIN_ACCEPT_DELAY_2;
  mibReq.Param.JoinAcceptDelay2 = JOIN_ACCEPT_DELAY2_MS;
  st = LoRaMacMibSetRequestConfirm(&mibReq);
  if (st != LORAMAC_STATUS_OK)
  {
    return false;
  }

  mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
  mibReq.Param.SystemMaxRxError = RX_TIMING_ERROR_MS;
  st = LoRaMacMibSetRequestConfirm(&mibReq);
  if (st != LORAMAC_STATUS_OK)
  {
    return false;
  }

  mibReq.Type = MIB_MIN_RX_SYMBOLS;
  mibReq.Param.MinRxSymbols = RX_MIN_SYMBOLS;
  st = LoRaMacMibSetRequestConfirm(&mibReq);
  if (st != LORAMAC_STATUS_OK)
  {
    return false;
  }

  mibReq.Type = MIB_RX2_CHANNEL;
  mibReq.Param.Rx2Channel.Frequency = RX2_FREQUENCY_AS923;
  mibReq.Param.Rx2Channel.Datarate = RX2_DR_AS923;
  st = LoRaMacMibSetRequestConfirm(&mibReq);
  if (st != LORAMAC_STATUS_OK)
  {
    return false;
  }

  mibReq.Type = MIB_PUBLIC_NETWORK;
  mibReq.Param.EnablePublicNetwork = true;
  st = LoRaMacMibSetRequestConfirm(&mibReq);
  if (st != LORAMAC_STATUS_OK)
  {
    return false;
  }

  return true;
}

static void AppRequestJoin(void)
{
  MlmeReq_t mlmeReq = {0};
  LoRaMacStatus_t reqStatus;

  mlmeReq.Type = MLME_JOIN;
  mlmeReq.Req.Join.NetworkActivation = ACTIVATION_TYPE_OTAA;
  mlmeReq.Req.Join.Datarate = APP_JOIN_DATARATE;
  AppJoinAttemptCount++;

  reqStatus = LoRaMacMlmeRequest(&mlmeReq);

  if (reqStatus == LORAMAC_STATUS_OK)
  {
    AppJoinRequested = true;
    AppJoinRequestTick = HAL_GetTick();
  }
  else
  {
    AppJoinRequested = false;
    AppNextJoinTime = HAL_GetTick() + APP_JOIN_RETRY_MS;
  }
}

static void AppSendUplink(void)
{
  McpsReq_t mcpsReq = {0};
  LoRaMacStatus_t reqStatus;

  uint16_t busVoltageMv = 0U;
  int16_t currentMa = 0;
  if (AppIna219Present)
  {
    (void)INA219_ReadBusVoltage_mV(&busVoltageMv);
    (void)INA219_ReadCurrent_mA(&currentMa);
  }

  AppTxBuffer[0] = AppComputeBatteryLevel(busVoltageMv);
  AppTxBuffer[1] = (uint8_t)(busVoltageMv >> 8);
  AppTxBuffer[2] = (uint8_t)(busVoltageMv);
  AppTxBuffer[3] = (uint8_t)((uint16_t)currentMa >> 8);
  AppTxBuffer[4] = (uint8_t)((uint16_t)currentMa);

  int16_t temperatureCx10 = 0;
  uint16_t humidityPctx10 = 0;
  uint16_t lightRaw = 0U;
  uint16_t soilRaw = 0U;

  /* Sensor cluster is already powered and warmed up by AppProcess. */
  if (Dht22Read(&temperatureCx10, &humidityPctx10))
  {
    gDiagSnapshot.dhtOkCount++;
  }
  else
  {
    gDiagSnapshot.dhtFailCount++;
  }
  gDiagSnapshot.dhtLastStage = gDht22LastStage;
  lightRaw = AdcReadChannel(ADC_CHSELR_CHSEL1);
  soilRaw = AdcReadChannel(ADC_CHSELR_CHSEL2);
  gDiagSnapshot.lightRaw = lightRaw;
  gDiagSnapshot.soilRaw = soilRaw;

  AppTxBuffer[5] = (uint8_t)((uint16_t)temperatureCx10 >> 8);
  AppTxBuffer[6] = (uint8_t)((uint16_t)temperatureCx10);
  AppTxBuffer[7] = (uint8_t)(humidityPctx10 >> 8);
  AppTxBuffer[8] = (uint8_t)(humidityPctx10);
  AppTxBuffer[9] = (uint8_t)(lightRaw >> 8);
  AppTxBuffer[10] = (uint8_t)(lightRaw);
  AppTxBuffer[11] = (uint8_t)(soilRaw >> 8);
  AppTxBuffer[12] = (uint8_t)(soilRaw);
  AppTxBuffer[13] = AppIdleCurrentX100uA;

  mcpsReq.Type = MCPS_UNCONFIRMED;
  mcpsReq.Req.Unconfirmed.fPort = APP_TX_PORT;
  mcpsReq.Req.Unconfirmed.fBuffer = AppTxBuffer;
  mcpsReq.Req.Unconfirmed.fBufferSize = APP_PAYLOAD_SIZE;
  mcpsReq.Req.Unconfirmed.Datarate = APP_TX_DATARATE;

  gDiagSnapshot.uplinkReqCount++;
  reqStatus = LoRaMacMcpsRequest(&mcpsReq);
  gDiagSnapshot.lastMcpsRequestStatus = (uint32_t)reqStatus;

  if (reqStatus == LORAMAC_STATUS_OK)
  {
    uint32_t waitMs = mcpsReq.ReqReturn.DutyCycleWaitTime;

    if (waitMs < APP_TX_PERIOD_MS)
    {
      waitMs = APP_TX_PERIOD_MS;
    }

    AppNextTxTime = HAL_GetTick() + waitMs;
    AppFrameCounter++;
  }
  else
  {
    AppNextTxTime = HAL_GetTick() + APP_TX_PERIOD_MS;
  }
}

static void AppProcess(void)
{
  uint32_t now = HAL_GetTick();
  bool macBusy;

  if (!AppJoined)
  {
    if (AppJoinRequested)
    {
      if ((now - AppJoinRequestTick) >= APP_JOIN_CONFIRM_WAIT_MS)
      {
        AppJoinRequested = false;
        AppNextJoinTime = now + APP_JOIN_RETRY_MS;
      }
    }

    if ((!AppJoinRequested) && (now >= AppNextJoinTime) && (!LoRaMacIsBusy()))
    {
      AppRequestJoin();
    }
    return;
  }

  macBusy = LoRaMacIsBusy();

  if (macBusy)
  {
    gDiagSnapshot.macBusyCount++;
    return;
  }

  if (!AppSensorPowerOn)
  {
    if (now < AppNextTxTime)
    {
      return;
    }

    /* Sample the idle current right before waking the sensor cluster, while
     * it's still off and the radio is asleep - the closest this firmware
     * gets to a "resting" reading without a STOP-mode rework. */
    if (AppIna219Present)
    {
      int16_t idleCurrent100uA = 0;

      if (INA219_ReadCurrent_x100uA(&idleCurrent100uA))
      {
        if (idleCurrent100uA < 0)
        {
          idleCurrent100uA = 0;
        }
        else if (idleCurrent100uA > 255)
        {
          idleCurrent100uA = 255;
        }
        AppIdleCurrentX100uA = (uint8_t)idleCurrent100uA;
      }
    }

    /* Power the sensor cluster and let it warm up across several main-loop
     * iterations instead of blocking TimerProcess()/LoRaMacProcess().
     * MOSFET gate is driven inverted: RESET conducts (sensor VCC ~3.3V),
     * SET starves it down to ~1.5V. Confirmed by measurement. */
    HAL_GPIO_WritePin(SENSOR_PWR_GPIO_Port, SENSOR_PWR_Pin, GPIO_PIN_RESET);
    AppSensorPowerOnTick = now;
    AppSensorPowerOn = true;
    return;
  }

  if ((now - AppSensorPowerOnTick) < SENSOR_POWERON_DELAY_MS)
  {
    return;
  }

  AppSendUplink();
  HAL_GPIO_WritePin(SENSOR_PWR_GPIO_Port, SENSOR_PWR_Pin, GPIO_PIN_SET);
  AppSensorPowerOn = false;
}

static bool AppRadioSanityCheck(void)
{
  uint32_t attempt;

  for (attempt = 0U; attempt < APP_RADIO_PROBE_RETRIES; attempt++)
  {
    SX1276Reset();
    HAL_Delay(2);

    if (SX1276Read(0x42U) == 0x12U)
    {
      return true;
    }

    HAL_Delay(APP_RADIO_PROBE_DELAY_MS);
  }

  return false;
}

static void AppInit(void)
{
  LoRaMacStatus_t macStatus;

  /* .noinit is not zeroed by the linker; reset the diagnostic snapshot
   * ourselves so leftover SRAM garbage doesn't taint the counters. */
  if (gDiagSnapshot.magic != APP_DIAG_MAGIC)
  {
    memset((void *)&gDiagSnapshot, 0, sizeof(gDiagSnapshot));
  }

  BoardInitMcu();
  RtcBoardInit();
  SX1276IoInit();

  AppRadioSanityCheck();

  AppIna219Present = INA219_Init(&hi2c1);
  Dht22Init();

  AppPrimitives.MacMcpsConfirm = AppMacMcpsConfirm;
  AppPrimitives.MacMcpsIndication = AppMacMcpsIndication;
  AppPrimitives.MacMlmeConfirm = AppMacMlmeConfirm;
  AppPrimitives.MacMlmeIndication = AppMacMlmeIndication;

  AppCallbacks.GetBatteryLevel = AppGetBatteryLevel;
  AppCallbacks.GetTemperatureLevel = AppGetTemperatureLevel;
  AppCallbacks.NvmDataChange = AppNvmDataChange;
  AppCallbacks.MacProcessNotify = AppMacProcessNotify;

  macStatus = LoRaMacInitialization(&AppPrimitives, &AppCallbacks, LORAWAN_ACTIVE_REGION);
  if (macStatus != LORAMAC_STATUS_OK)
  {
    Error_Handler();
  }

  if (!AppConfigureMac())
  {
    Error_Handler();
  }

  macStatus = LoRaMacStart();
  if (macStatus != LORAMAC_STATUS_OK)
  {
    Error_Handler();
  }

  AppNextJoinTime = HAL_GetTick();
  AppJoined = false;
  AppJoinRequested = false;
  AppNextTxTime = 0;
  AppFrameCounter = 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  AppInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    TimerProcess();
    LoRaMacProcess();
    AppProcess();
    gDiagSnapshot.mainLoopCount++;
    AppUpdateDiagSnapshot();
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
