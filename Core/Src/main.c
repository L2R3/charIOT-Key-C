/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// display
#include "cmsis_os2.h"
#include "csrc/u8g2.h"
#include "stm32l4xx_hal_dac.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_it.h"

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

CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for scanKeys */
osThreadId_t scanKeysHandle;
const osThreadAttr_t scanKeys_attributes = {
  .name = "scanKeys",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for displayUpdate */
osThreadId_t displayUpdateHandle;
const osThreadAttr_t displayUpdate_attributes = {
  .name = "displayUpdate",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for synthesize */
osThreadId_t synthesizeHandle;
const osThreadAttr_t synthesize_attributes = {
  .name = "synthesize",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for keysMutex */
osMutexId_t keysMutexHandle;
const osMutexAttr_t keysMutex_attributes = {
  .name = "keysMutex"
};
/* USER CODE BEGIN PV */

const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

u8g2_t u8g2;

const float fs = 22000;
const float fA = 440;
float dreal[12];
float dimag[12];

volatile uint16_t keys = 0x0FFF;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void scanKeysTask(void *argument);
void displayUpdateTask(void *argument);
void synthesizeTask(void *argument);

/* USER CODE BEGIN PFP */

void serialPrint(char val[]);
void serialPrintln(char val[]);
void delayMicro(uint16_t us);
void synthesize_waves(int index);

void setOutMuxBit(const uint8_t bitIdx, const bool value);
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr);
uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

void setRow(uint8_t rowIdx);
uint8_t readCols();
uint16_t readKeys();

void rotationSteps(float *dreal, float *dimag);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



#define PI 3.14159265359

#define ROOT_12_OF_2 1.05946

#define C_samples 337
#define C_sharp_samples (uint32_t)(C_samples /       ROOT_12_OF_2)
#define D_samples       (uint32_t)(C_sharp_samples / ROOT_12_OF_2)
#define D_sharp_samples (uint32_t)(D_samples /       ROOT_12_OF_2)
#define E_samples       (uint32_t)(D_sharp_samples / ROOT_12_OF_2)
#define F_samples       (uint32_t)(E_samples /       ROOT_12_OF_2)
#define F_sharp_samples (uint32_t)(F_samples /       ROOT_12_OF_2)
#define G_samples       (uint32_t)(F_sharp_samples / ROOT_12_OF_2)
#define G_sharp_samples (uint32_t)(G_samples /       ROOT_12_OF_2)
#define A_samples       (uint32_t)(G_sharp_samples / ROOT_12_OF_2)
#define A_sharp_samples (uint32_t)(A_samples /       ROOT_12_OF_2)
#define B_samples       (uint32_t)(A_sharp_samples / ROOT_12_OF_2)


#define OUTPUT_SAMPLES C_samples 
uint16_t output_LUT[OUTPUT_SAMPLES];


const uint32_t sample_counts [12] = {
    C_samples,
    C_sharp_samples,
    D_samples,
    D_sharp_samples,
    E_samples,
    F_samples,
    F_sharp_samples,
    G_samples,
    G_sharp_samples,
    A_samples,
    A_sharp_samples,
    B_samples
};

int16_t C_LUT          [C_samples];
int16_t C_sharp_LUT    [C_sharp_samples];
int16_t D_LUT          [D_samples];
int16_t D_sharp_LUT    [D_sharp_samples];
int16_t E_LUT          [E_samples];
int16_t F_LUT          [F_samples];
int16_t F_sharp_LUT    [F_sharp_samples];
int16_t G_LUT          [G_samples];
int16_t G_sharp_LUT    [G_sharp_samples];
int16_t A_LUT          [A_samples];
int16_t A_sharp_LUT    [A_sharp_samples];
int16_t B_LUT          [B_samples];

int16_t* lookup_tables [12] = {
    C_LUT,
    C_sharp_LUT,
    D_LUT,
    D_sharp_LUT,
    E_LUT,
    F_LUT,
    F_sharp_LUT,
    G_LUT,
    G_sharp_LUT,
    A_LUT,
    A_sharp_LUT,
    B_LUT,
};

uint16_t lookup_indices [12];

uint16_t DMAkeys;

inline void synthesize_waves(int index){

    int32_t out = 0;

    HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
    for (int t = 0; t < 12; t++){
        lookup_indices[t] = (lookup_indices[t] + 4) % sample_counts[t];
        bool key_pressed = ~DMAkeys & ( 1 << t);
        out += key_pressed ? lookup_tables[t][lookup_indices[t]] : 0;
    }
    output_LUT[index] = ((uint16_t)(out >> 3)) + 2048;

    HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
}


void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac){

    //serialPrintln("half");
    DMAkeys = __atomic_load_n(&keys, __ATOMIC_RELAXED);
    

    for (int i = 0; i < OUTPUT_SAMPLES/2; i++) {
        synthesize_waves(i);
    }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    
    for (int i = OUTPUT_SAMPLES/2; i < OUTPUT_SAMPLES; i++) {
        synthesize_waves(i);
    }
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

        rotationSteps(dreal, dimag);

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim7);

    HAL_TIM_Base_Start_IT(&htim6);


    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)output_LUT, OUTPUT_SAMPLES, DAC_ALIGN_12B_R);
//    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t *)output_LUT, OUTPUT_SAMPLES, DAC_ALIGN_12B_R);
//    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

    setOutMuxBit(DRST_BIT, GPIO_PIN_RESET);
    delayMicro(2);
    setOutMuxBit(DRST_BIT, GPIO_PIN_SET);
    u8g2_Setup_ssd1305_i2c_128x32_noname_f(&u8g2, U8G2_R0, u8x8_byte_i2c,
                u8x8_gpio_and_delay);
    u8g2_InitDisplay(&u8g2);
    u8g2_ClearDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    setOutMuxBit(DEN_BIT, GPIO_PIN_SET);

    serialPrintln("charIOT-Key-C");


    //Generate wave tables
    char buf [20];
    for (int t = 0; t < 12; t++) {

        uint32_t samples =  sample_counts[t];

        sprintf(buf, "\n\n Lut: %i------", t);
        //serialPrintln(buf);


//      PURE SINE WAVES
//        for (int i = 0; i < samples; i++) {
//            lookup_tables[t][i] = 2048 * sin(2.0 * PI * (float)i / (float) samples);
//            sprintf(buf, "%i %i ", i, lookup_tables[t][i]);
//            //serialPrintln(buf);
//        }

//      SQUARE WAVES
//        int half_samples = samples / 2;
//		for (int i = 0; i < samples; i++) {
//			lookup_tables[t][i] = (i <= half_samples) ? 2048 * (1.0) : 2048 * (-1.0);
//			sprintf(buf, "%i %i ", i, lookup_tables[t][i]);
//			//serialPrintln(buf);
//		}

////      SAWTOOTH WAVES
//        int half_samples = samples / 2;
//		for (int i = 0; i < samples; i++) {
//			lookup_tables[t][i] = (i <= half_samples) ? 2048 * ((float)(i-half_samples) / (float) samples) : 2048 * ((float)(i-half_samples) / (float) samples);
//			sprintf(buf, "%i %i ", i, lookup_tables[t][i]);
//			//serialPrintln(buf);
//		}

////      TRIANGLE WAVES
//        int half_samples = samples / 2;
//        int first_fourth = samples / 4;
//        int third_fourth = half_samples + first_fourth;
//		for (int i = 0; i < samples; i++) {
//			lookup_tables[t][i] = (i <= first_fourth) ?
//									2048 * ((float)(-i) / (float) samples)
//								:
//									(i <= third_fourth) ? 2048 * ((float)(i-half_samples) / (float) samples) : 2048 * ((float)(samples-i) / (float) samples);
//			sprintf(buf, "%i %i ", i, lookup_tables[t][i]);
//			//serialPrintln(buf);
//		}

////      TRUMPET (SQUARE + SINE) WAVES - HMMMMMM
		int half_samples = samples / 2;
		for (int i = 0; i < samples; i++) {
			lookup_tables[t][i+1] 	= (i <= half_samples) ? (1024 *(sin(2.0 * PI * (float)i / (float) samples)) + 1024*(1.0))
													      : (1024 *(sin(2.0 * PI * (float)i / (float) samples)) + 1024*(-1.0));
//			sprintf(buf, "%i %i ", i, lookup_tables[t][i]);
			//serialPrintln(buf);
		}

    }



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of keysMutex */
  keysMutexHandle = osMutexNew(&keysMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
        /* add mutexes, ... */
        osMutexRelease(keysMutexHandle);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of scanKeys */
  scanKeysHandle = osThreadNew(scanKeysTask, NULL, &scanKeys_attributes);

  /* creation of displayUpdate */
  displayUpdateHandle = osThreadNew(displayUpdateTask, NULL, &displayUpdate_attributes);

  /* creation of synthesize */
  synthesizeHandle = osThreadNew(synthesizeTask, NULL, &synthesize_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
        /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
        /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 40;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00300F33;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Period = 1814;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3636-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 80-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(REN_GPIO_Port, REN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RA0_Pin|RA1_Pin|LED_BUILTIN_Pin|RA2_Pin
                          |OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : C0_Pin C2_Pin C1_Pin C3_Pin */
  GPIO_InitStruct.Pin = C0_Pin|C2_Pin|C1_Pin|C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : REN_Pin */
  GPIO_InitStruct.Pin = REN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(REN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RA0_Pin RA1_Pin LED_BUILTIN_Pin RA2_Pin
                           OUT_Pin */
  GPIO_InitStruct.Pin = RA0_Pin|RA1_Pin|LED_BUILTIN_Pin|RA2_Pin
                          |OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void serialPrint(char val[]) {
	HAL_UART_Transmit(&huart2, (uint8_t*) val, strlen(val), 10);
}

void serialPrintln(char val[]) {

	HAL_UART_Transmit(&huart2, (uint8_t*) val, strlen(val), 10);
	char newline[2] = "\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*) newline, 2, 10);

}

void delayMicro(uint16_t us) {

	htim7.Instance->CNT = 0;
	while (htim7.Instance->CNT < us)
		;

}

void setOutMuxBit(const uint8_t bitIdx, const bool value) {

	HAL_GPIO_WritePin(REN_GPIO_Port, REN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RA0_GPIO_Port, RA0_Pin, bitIdx & 0x01);
	HAL_GPIO_WritePin(RA1_GPIO_Port, RA1_Pin, bitIdx & 0x02);
	HAL_GPIO_WritePin(RA2_GPIO_Port, RA2_Pin, bitIdx & 0x04);
	HAL_GPIO_WritePin(OUT_GPIO_Port, OUT_Pin, value);
	HAL_GPIO_WritePin(REN_GPIO_Port, REN_Pin, GPIO_PIN_SET);
	delayMicro(2);
	HAL_GPIO_WritePin(REN_GPIO_Port, REN_Pin, GPIO_PIN_RESET);

}

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr) {

	return 1;

}

uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {

	static uint8_t buffer[32];
	static uint8_t buf_idx;
	uint8_t *data;

	switch (msg) {
	case U8X8_MSG_BYTE_SEND:
		data = (uint8_t*) arg_ptr;
		while (arg_int > 0) {
			buffer[buf_idx++] = *data;
			data++;
			arg_int--;
		}
		break;
	case U8X8_MSG_BYTE_INIT:
		break;
	case U8X8_MSG_BYTE_SET_DC:
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		buf_idx = 0;
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		HAL_I2C_Master_Transmit(&hi2c1, u8x8_GetI2CAddress(u8x8),
				(uint8_t*) buffer, buf_idx, HAL_MAX_DELAY);
		break;
	default:
		return 0;
	}

	return 1;

}

void setRow(uint8_t rowIdx) {

	HAL_GPIO_WritePin(REN_GPIO_Port, REN_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(RA0_GPIO_Port, RA0_Pin, rowIdx & 0x01);
	HAL_GPIO_WritePin(RA1_GPIO_Port, RA1_Pin, rowIdx & 0x02);
	HAL_GPIO_WritePin(RA2_GPIO_Port, RA2_Pin, rowIdx & 0x04);

	HAL_GPIO_WritePin(REN_GPIO_Port, REN_Pin, GPIO_PIN_SET);

}

uint8_t readCols() {

	uint8_t C0_val = HAL_GPIO_ReadPin(C0_GPIO_Port, C0_Pin);
	uint8_t C1_val = HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin);
	uint8_t C2_val = HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin);
	uint8_t C3_val = HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin);

	return (C3_val << 3) | (C2_val << 2) | (C1_val << 1) | C0_val;

}

uint16_t readKeys() {

	uint16_t keysRead = 0;

	for (int i = 0; i <= 2; i++) {

		setRow(i);
		delayMicro(5);
		keysRead |= readCols() << (4 * i);

	}

	return keysRead;

}

void rotationSteps(float *dreal, float *dimag) {

	float phi;

	for (int i = 0; i < 12; i++) {

		phi = 2 * M_PI * fA * pow(2, (i - 9) / 12.0) / fs;
		dreal[i] = cos(phi);
		dimag[i] = sin(phi);

	}

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_scanKeysTask */
/**
 * @brief Function implementing the scanKeys thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_scanKeysTask */
void scanKeysTask(void *argument)
{
  /* USER CODE BEGIN scanKeysTask */

	const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	for (;;) {

		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		uint16_t localKeys = readKeys();

		osMutexAcquire(keysMutexHandle, osWaitForever);

		__atomic_store_n(&keys, localKeys, __ATOMIC_RELAXED);

		osMutexRelease(keysMutexHandle);

	}
  /* USER CODE END scanKeysTask */
}

/* USER CODE BEGIN Header_displayUpdateTask */
/**
 * @brief Function implementing the displayUpdate thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_displayUpdateTask */
void displayUpdateTask(void *argument)
{
  /* USER CODE BEGIN displayUpdateTask */

    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* Infinite loop */
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        osMutexAcquire(keysMutexHandle, osWaitForever);

        uint16_t localKeys = __atomic_load_n(&keys, __ATOMIC_RELAXED);

        osMutexRelease(keysMutexHandle);

        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);

        if (localKeys == 0x0FFF) {
                u8g2_DrawStr(&u8g2, 2, 20, "- ^_^ -");
        } else {
                u8g2_DrawStr(&u8g2, 2, 20, "- ^0^ -");
        }

        u8g2_SendBuffer(&u8g2);

    }

  /* USER CODE END displayUpdateTask */
}

/* USER CODE BEGIN Header_synthesizeTask */
/**
* @brief Function implementing the synthesize thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_synthesizeTask */
void synthesizeTask(void *argument)
{
  /* USER CODE BEGIN synthesizeTask */
  /* Infinite loop */


    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 10000 / portTICK_PERIOD_MS;

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint16_t localKeys = __atomic_load_n(&keys, __ATOMIC_RELAXED);

        for(int i = 0; i < OUTPUT_SAMPLES; i++){
            char buf [20];
            sprintf(buf, "%i ", output_LUT[i]);
            serialPrint(buf);
        }
        serialPrintln("\n\n");



    }
  /* USER CODE END synthesizeTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    /*

	if (htim == &htim6) {

		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

		static float real[12] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
		static float imag[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

		float real2;
		float imag2;

		float Vadd = 0;

		uint16_t localKeys;

		localKeys = __atomic_load_n(&keys, __ATOMIC_RELAXED);

		for (int i = 0; i < 12; i++) {

			if (!(localKeys & 1)) {

				real2 = dreal[i] * real[i] - dimag[i] * imag[i];
				imag2 = dimag[i] * real[i] + dreal[i] * imag[i];

				real[i] = real2;
				imag[i] = imag2;

				Vadd += real[i];

			}

			localKeys >>= 1;

		}

		int16_t Vout = (int16_t) 1024 * Vadd / 12.0;

		HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, Vout + 2048);
		HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, Vout + 2048);

		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin,
				GPIO_PIN_RESET);

	}
        */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
	while (1) {
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
