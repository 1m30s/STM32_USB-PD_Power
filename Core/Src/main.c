/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  * [23/06/28] Modified by H.Yamada
  * rev_c1815@yahoo.co.jp
  *
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usbpd.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "AQM1248_Driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

uint8_t  g_receivedPDOSize, g_receivedFirstAPDOIdx;
USBPD_PDO_TypeDef g_receivedPDO[16];

uint8_t g_curRequestPDOIndex; //  (object position of PDO) - 1
// g_curRequestPDOIndex is determined and updated by USBPD_DPM_SNK_EvaluateCapabilities() function.
// do not change at main routine. just change g_fixedVoltageLevel

USBPD_SNKRDO_TypeDef g_curActiveRDO;
uint8_t  g_isPPSMode; // set when launched or PDO received
uint16_t g_fixedVoltageLevel; // mV
uint16_t g_ppsVoltageRange[2], g_ppsVoltageLevel; // min, max. in mV

volatile uint8_t g_ledState;
uint8_t g_srcPDOReceived;

void SetPPSVoltageLevel(uint16_t newLevel)
{
	g_ppsVoltageLevel = newLevel;
	// store to EEPROM
}
/*
void CurRDO_SetVoltage(uint16_t newVoltage)
{
	if(!IS_PPS_MODE()) return;
	g_curActiveRDO.ProgRDO.OutputVoltageIn20mV = newVoltage/20;
}*/
uint16_t CurRDO_GetVoltage()
{
	if(!IS_PPS_MODE()) {
		return g_receivedPDO[g_curRequestPDOIndex].SRCFixedPDO.VoltageIn50mVunits * 50;
	}
	else
	{
		return g_curActiveRDO.ProgRDO.OutputVoltageIn20mV * 20;
	}
}

uint16_t CurRDO_GetCurrent()
{
	if(!IS_PPS_MODE()) {
		return g_receivedPDO[g_curRequestPDOIndex].SRCFixedPDO.MaxCurrentIn10mAunits * 10;
	}
	else
	{
		return g_curActiveRDO.ProgRDO.OperatingCurrentIn50mAunits * 50;
	}
}

//#define VREF_VOLTAGE 3300
static uint16_t GetVREFVoltage()
{
	// VREF+: unknown, range of 2.0V - 3.3V
	uint16_t vrefVoltage = 2000;
	if(g_adcData[3]){
		vrefVoltage = 1212 * 4096 / g_adcData[3];
	}
	return vrefVoltage;
}
#define DIVIDER_RATIO 11 // External voltage divider ratio
uint16_t GetVBUSVoltage() // result is in 10 mV scale
{

	uint16_t ret = (uint32_t)g_adcData[0] * GetVREFVoltage() / 4096 * DIVIDER_RATIO / 10;
	return ret;
}
#define CURRENT_AMP_RATIO 1 // 1V/1A
uint16_t GetVBUSCurrent() // result is in 1 mA scale
{
	uint16_t ret = (uint32_t)g_adcData[2] * GetVREFVoltage() / 4096 * CURRENT_AMP_RATIO;
	return ret;
}

USBPD_StatusTypeDef SendRequest()
{
//	return USBPD_DPM_RequestMessageRequest(0, g_curRequestPDOIndex, 0 ); // Request
	return USBPD_DPM_RequestGetSourceCapability(0); // Source_Cap -> Request
}

// EEPROM Load & Save
struct sEEPData
{
	uint8_t ppsMode; // last time was it pps mode ?
	uint16_t ppsVoltage;
	uint16_t fixedVoltage;
};
// EEPROM AT93C56 (256x8)
// Command length: (4bit 0) + 3bit command + 9bit address + 8bit data
void EEPROM_WriteEnable()
{
	uint8_t txData[3] = {0x09, 0x80, 0x00};
	HAL_GPIO_WritePin(EEP_CS_GPIO_Port, EEP_CS_Pin, GPIO_PIN_SET);
	HAL_SPI_Transmit(&hspi2, txData, 3, 100);
//	HAL_SPI_Transmit_IT(&hspi2, txData, 3);
	HAL_GPIO_WritePin(EEP_CS_GPIO_Port, EEP_CS_Pin, GPIO_PIN_RESET);
}
void EEPROM_WriteDisable()
{
	uint8_t txData[3] = {0x08, 0x00, 0x00};
	HAL_GPIO_WritePin(EEP_CS_GPIO_Port, EEP_CS_Pin, GPIO_PIN_SET);
	HAL_SPI_Transmit(&hspi2, txData, 3, 100);
//	HAL_SPI_Transmit_IT(&hspi2, txData, 3);
	HAL_GPIO_WritePin(EEP_CS_GPIO_Port, EEP_CS_Pin, GPIO_PIN_RESET);
}
void EEPROM_Write(uint16_t addr, uint8_t data)
{
	HAL_GPIO_WritePin(EEP_CS_GPIO_Port, EEP_CS_Pin, GPIO_PIN_SET);
	uint8_t txData[3] = {0x0A | ((addr >> 8) & 1), addr, data};
//	uint8_t txData[3] = {0xA0 | 0, data>>4, data<<4};
//	uint8_t txData[3] = {0x0A, 0, data};
	HAL_SPI_Transmit(&hspi2, txData, 3, 100);
//	HAL_SPI_Transmit_IT(&hspi2, txData, 3);

	HAL_GPIO_WritePin(EEP_CS_GPIO_Port, EEP_CS_Pin, GPIO_PIN_RESET);
	// tWP: max. 10ms
	osDelay(10);
}
uint8_t EEPROM_Read(uint16_t addr)
{
	HAL_GPIO_WritePin(EEP_CS_GPIO_Port, EEP_CS_Pin, GPIO_PIN_SET);
	uint8_t txData[3] = {(0x0C<<1) | ((addr >> 7) & 3), (addr << 1), 0};
//	uint8_t txData[3] = {0xC0 | ((addr >> 8) & 1), addr, 0};
	uint8_t rxData[3];
	HAL_SPI_TransmitReceive(&hspi2, txData, rxData, 3, 100);
//	HAL_SPI_TransmitReceive_IT(&hspi2, txData, rxData, 3);

	HAL_GPIO_WritePin(EEP_CS_GPIO_Port, EEP_CS_Pin, GPIO_PIN_RESET);
	return rxData[2];
//	return (rxData[1] << 4) | (rxData[2] >> 4);
}
void EEPROM_Save(const struct sEEPData* pData)
{
	// set CPOL=0, CPHA=0
//	uint32_t cr1_restore = hspi2.Instance->CR1;
//	hspi2.Instance->CR1 &= ~0x03;
//	HAL_SPI_Transmit(&hspi2, (uint8_t*)pData, 1, 100); // dummy

	EEPROM_WriteEnable();

	// update checksum
	uint8_t cs = 0;
	uint8_t* p = (uint8_t*)pData;
	uint16_t i;
	for(i=0; i<sizeof(struct sEEPData); i++) cs += p[i];
	EEPROM_Write(0x00, ~cs);

	for(i=0; i<sizeof(struct sEEPData); i++)
	{
		EEPROM_Write(0x02 + i, p[i]);
	}

	EEPROM_WriteDisable();

	// restore
//	hspi2.Instance->CR1 = cr1_restore;
//	HAL_SPI_Transmit(&hspi2, (uint8_t*)pData, 1, 100); // dummy
}
uint8_t g_cs;
uint8_t EEPROM_Load(struct sEEPData* pData)
{
	// set CPOL=0, CPHA=0
//	uint32_t cr1_restore = hspi2.Instance->CR1;
//	hspi2.Instance->CR1 &= ~0x03;
//	HAL_SPI_Transmit(&hspi2, (uint8_t*)pData, 1, 100); // dummy

	struct sEEPData temp;
	uint8_t cs_read = ~EEPROM_Read(0x00);
	uint16_t i;
	uint8_t cs = 0;
	uint8_t* p = (uint8_t*)&temp;
	for(i=0; i<sizeof(temp); i++)
	{
		cs += p[i] = EEPROM_Read(0x02 + i);
	}

	g_cs = cs_read;

	// restore
//	hspi2.Instance->CR1 = cr1_restore;
//	HAL_SPI_Transmit(&hspi2, (uint8_t*)pData, 1, 100); // dummy

	// Checksum
	if(cs == cs_read) *pData = temp;
	else return -1;
	return 0;
}

// Switch handler
void SwitchHandler(uint8_t swState)
{
	static uint8_t outEnable;

	char text[64];
	switch(swState)
	{
	case (SW_SW1 | SW_LONG):
		outEnable ^= 1;
//		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,
//				(outEnable)? GPIO_PIN_SET: GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VBUS_EN_GPIO_Port, VBUS_EN_Pin,
				(outEnable)? GPIO_PIN_SET: GPIO_PIN_RESET);
		break;
	case (SW_SW2 | SW_PUSHING):
		if(IS_PPS_MODE())
		{
			// PPS
			if(HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == GPIO_PIN_RESET ||
					outEnable == 0) // Voltage change is allowed
			{
				// Increase voltage
				uint16_t newVoltage = (g_ppsVoltageLevel + 100)/100 * 100;
				if(newVoltage <= g_ppsVoltageRange[1])
				{
					SetPPSVoltageLevel(newVoltage);
					SendRequest();
					//USBPD_DPM_RequestGetSourceCapability(0); //
				}
			}
		}
		break;
	case SW_SW2: // +
		if(IS_PPS_MODE())
		{
			// PPS
			if(HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == GPIO_PIN_RESET ||
					outEnable == 0) // Voltage change is allowed
			{
				// Increase voltage
				uint16_t newVoltage = g_ppsVoltageLevel + 20;
				if(newVoltage <= g_ppsVoltageRange[1])
				{
					SetPPSVoltageLevel(newVoltage);
					SendRequest();
					//USBPD_DPM_RequestGetSourceCapability(0); //
				}
			}
		}
		else
		{
			// Fixed voltage
			if(outEnable == 0) // Voltage change is allowed
			{
				// Change VBUS Voltage
				if(g_curRequestPDOIndex < g_receivedPDOSize - 1)
				{
					//g_curRequestPDOIndex = (g_curRequestPDOIndex + 1) % g_receivedFirstAPDOIdx;
					if(g_receivedPDO[g_curRequestPDOIndex + 1].GenericPDO.PowerObject == USBPD_CORE_PDO_TYPE_FIXED)
					{
						g_fixedVoltageLevel = g_receivedPDO[g_curRequestPDOIndex + 1].SRCFixedPDO.VoltageIn50mVunits * 50;// just change g_fixedVoltageLevel

						// Requst to send Request message
						SendRequest();
					}
				}
				// Request to send GetSouceCapability for new evaluates
				//USBPD_DPM_RequestGetSourceCapability(0); //
			}
		}
	//	sprintf(text, "PDO Size=%d, PDO[0] = %08X",
	//			g_receivedPDOSize, g_receivedPDO[0]);
	//	USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, text, strlen(text)); // trace test
		break;
	case (SW_SW3 | SW_PUSHING):
		if(IS_PPS_MODE())
		{
			// PPS
			if(HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == GPIO_PIN_RESET ||
					outEnable == 0) // Voltage change is allowed
			{
				// Increase voltage
				uint16_t newVoltage = (g_ppsVoltageLevel - 100) / 100 * 100;
				if(newVoltage >= g_ppsVoltageRange[0])
				{
					SetPPSVoltageLevel(newVoltage);
					SendRequest();
				}
			}
		}
		break;
	case SW_SW3: // -
		if(IS_PPS_MODE())
		{
			// PPS
			if(HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == GPIO_PIN_RESET ||
					outEnable == 0) // Voltage change is allowed
			{
				// Increase voltage
				uint16_t newVoltage = g_ppsVoltageLevel - 20;
				if(newVoltage >= g_ppsVoltageRange[0])
				{
					SetPPSVoltageLevel(newVoltage);
					SendRequest();
				}
			}
		}
		else
		{
			// Fixed voltage
			if(outEnable == 0) // Voltage change is allowed
			{
				// Change VBUS Voltage
				if(g_curRequestPDOIndex)
				{
					// change request voltage level
					g_fixedVoltageLevel = g_receivedPDO[g_curRequestPDOIndex - 1].SRCFixedPDO.VoltageIn50mVunits * 50;

					// Requst to send Request message
					SendRequest();
				}
				//g_curRequestPDOIndex = g_curRequestPDOIndex? (g_curRequestPDOIndex - 1): 0;

				// Request to send GetSouceCapability for new evaluates
				//USBPD_DPM_RequestGetSourceCapability(0); //
			}
		}
		break;
	case SW_SW4:
		g_dispInfo.displayState ^= 1;

		// Show ADC Data
		//sprintf(text, "ADC=%d,%d,%d,%d",
		//		g_adcData[0], g_adcData[1], g_adcData[2], g_adcData[3]);
		//USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, text, strlen(text)); // trace test

		// Show VBUS Voltage
		/*
		{
			uint16_t vbus = GetVBUSVoltage();
			sprintf(text, "VBUS = %01d.%02dV",
					vbus/100, vbus%100);
		}
		USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, text, strlen(text)); // trace test
		*/

		// Change VBUS Voltage
		//g_curRequestPDOIndex ^= 1;
		// Requst to send Request message
		//USBPD_DPM_RequestMessageRequest(0, g_curRequestPDOIndex, 0 );
		//USBPD_DPM_RequestGetSourceCapability(0); //

		break;
	}
	// Store new voltage level to eeprom
	if((swState & 0xF) == SW_SW3 || (swState & 0xF) == SW_SW2) // - or +
	{
		struct sEEPData eep;
		eep.ppsMode = g_isPPSMode;
		eep.fixedVoltage = g_fixedVoltageLevel;
		eep.ppsVoltage   = g_ppsVoltageLevel;
		EEPROM_Save(&eep);
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UCPD2_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

uint32_t g_lastTick;
volatile uint8_t g_swState;

struct sPDDisplayInfo g_dispInfo;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_UCPD2_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  g_dispInfo.curVoltage = 0;
  g_dispInfo.curCurrent = 0;
  g_dispInfo.pdVersion = 2;
  g_dispInfo.pdStatus = 0;
  g_dispInfo.displayState = 0;
  g_dispInfo.ccStatus = 0;

  g_ledState = LED_STATE_BLINK_RED;

  // call before ADC conversion has started
  HAL_ADCEx_Calibration_Start(&hadc1);

  // Start Timer 2 in Interrupt Mode (invokes ADC starting)
  // (Timer does not start unless user writes code for it)
  HAL_TIM_Base_Start_IT(&htim2);

  AQM1248_Init();

  g_lastTick = HAL_GetTick();
  if(1)
  {
	  struct sEEPData eep;
	  if(EEPROM_Load(&eep) == 0)
	  {
		  g_ppsVoltageLevel = eep.ppsVoltage;
		  g_fixedVoltageLevel = eep.fixedVoltage;
	  }
	  else
	  {
		AQM1248_Clear();
		//                123456789012345678901234567890
		AQM1248_DrawText("EEPROM load failed.", 0, 2);
		  while(HAL_GetTick() < g_lastTick + 1000);
	  }
  }
  else
  {

	  g_fixedVoltageLevel = 9000;
  }


  /* USER CODE END 2 */

  /* USBPD initialisation ---------------------------------*/
  MX_USBPD_Init();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3200000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UCPD2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD2_Init(void)
{

  /* USER CODE BEGIN UCPD2_Init 0 */

  /* USER CODE END UCPD2_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UCPD2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOD);
  /**UCPD2 GPIO Configuration
  PD0   ------> UCPD2_CC1
  PD2   ------> UCPD2_CC2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* UCPD2 DMA Init */

  /* UCPD2_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_UCPD2_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

  /* UCPD2_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_UCPD2_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* UCPD2 interrupt Init */
  NVIC_SetPriority(UCPD1_2_IRQn, 3);
  NVIC_EnableIRQ(UCPD1_2_IRQn);

  /* USER CODE BEGIN UCPD2_Init 1 */

  /* USER CODE END UCPD2_Init 1 */
  /* USER CODE BEGIN UCPD2_Init 2 */

  /* USER CODE END UCPD2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 3);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Ch4_7_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VBUS_EN_GPIO_Port, VBUS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|EEP_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW1_Pin SW4_Pin SW5_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW4_Pin|SW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_EN_Pin */
  GPIO_InitStruct.Pin = VBUS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VBUS_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LED1_Pin LED2_Pin EEP_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LED1_Pin|LED2_Pin|EEP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	uint8_t swState = g_swState;
	g_swState = 0;
#if LED_DEBUG_MODE
	if(swState){
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
	else
	{
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	}
#endif
//#define LED_STATE_RED			1
//#define LED_STATE_GREEN			2
//#define LED_STATE_ORANGE		3
//#define LED_STATE_BLINK_RED		(8 | LED_STATE_RED)
//#define LED_STATE_BLINK_GREEN	(8 | LED_STATE_GREEN)
//#define LED_STATE_BLINK_ORANGE	(8 | LED_STATE_ORANGE)

	if(g_srcPDOReceived)
	{
		// First PDO received
		struct sEEPData eep;
		if(EEPROM_Load(&eep) == 0)
		{
			if(eep.ppsMode && (g_isPPSMode == 0))
			{
				// last time was PPS mode and now is fixed voltage mode
				AQM1248_Clear();
				//                1234567890123456789012345
				AQM1248_DrawText("Last time was on PPS but", 0, 0);
				AQM1248_DrawText("connected source does not", 0, 1);
				AQM1248_DrawText(" support PPS.", 0, 2);
				AQM1248_DrawText("Push SW to continue", 0, 3);
				while(g_swState == 0);
				g_swState = 0;
				eep.ppsMode = 0;
				EEPROM_Save(&eep);
			}
		}
		g_srcPDOReceived = 0;
	}

	// Write LED pin
	if(g_ledState & LED_STATE_BLINK)
	{
		static uint8_t a;
		if(g_ledState & LED_STATE_RED)
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (a&1)? GPIO_PIN_SET: GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		}

		if(g_ledState & LED_STATE_GREEN)
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (a&1)? GPIO_PIN_SET: GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		}
		a++;
	}
	else if(g_ledState)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (g_ledState & LED_STATE_RED)? GPIO_PIN_SET: GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (g_ledState & LED_STATE_GREEN)? GPIO_PIN_SET: GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	}

	SwitchHandler(swState);

	g_dispInfo.curVoltage = GetVBUSVoltage() * 10;
	g_dispInfo.curCurrent = GetVBUSCurrent();

	AQM1248_UpdateDisplay(&g_dispInfo);

	osDelay(100); // 100ms delay
  }
  /* USER CODE END 5 */
}

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
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
