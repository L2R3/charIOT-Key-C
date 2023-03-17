/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// display
#include "cmsis_os2.h"
#include "csrc/u8g2.h"
#include "stm32l4xx_hal_dac.h"
#include "stm32l4xx_hal_gpio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	uint8_t Message[8];
	uint32_t ID;
} CAN_MSG_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PI 3.14159265359

#define ROOT_12_OF_2 1.05946

#define C_SAMPLES 337
#define C_SHARP_SAMPLES (uint32_t)(C_SAMPLES /       ROOT_12_OF_2)
#define D_SAMPLES       (uint32_t)(C_SHARP_SAMPLES / ROOT_12_OF_2)
#define D_SHARP_SAMPLES (uint32_t)(D_SAMPLES /       ROOT_12_OF_2)
#define E_SAMPLES       (uint32_t)(D_SHARP_SAMPLES / ROOT_12_OF_2)
#define F_SAMPLES       (uint32_t)(E_SAMPLES /       ROOT_12_OF_2)
#define F_SHARP_SAMPLES (uint32_t)(F_SAMPLES /       ROOT_12_OF_2)
#define G_SAMPLES       (uint32_t)(F_SHARP_SAMPLES / ROOT_12_OF_2)
#define G_SHARP_SAMPLES (uint32_t)(G_SAMPLES /       ROOT_12_OF_2)
#define A_SAMPLES       (uint32_t)(G_SHARP_SAMPLES / ROOT_12_OF_2)
#define A_SHARP_SAMPLES (uint32_t)(A_SAMPLES /       ROOT_12_OF_2)
#define B_SAMPLES       (uint32_t)(A_SHARP_SAMPLES / ROOT_12_OF_2)

#define OUTPUT_SAMPLES C_SAMPLES 



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
/* Definitions for decodeTask */
osThreadId_t decodeTaskHandle;
const osThreadAttr_t decodeTask_attributes = {
  .name = "decodeTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN_TX_TaskName */
osThreadId_t CAN_TX_TaskNameHandle;
const osThreadAttr_t CAN_TX_TaskName_attributes = {
  .name = "CAN_TX_TaskName",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for handshakeTask */
osThreadId_t handshakeTaskHandle;
const osThreadAttr_t handshakeTask_attributes = {
  .name = "handshakeTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for msgInQ */
osMessageQueueId_t msgInQHandle;
const osMessageQueueAttr_t msgInQ_attributes = {
  .name = "msgInQ"
};
/* Definitions for msgOutQ */
osMessageQueueId_t msgOutQHandle;
const osMessageQueueAttr_t msgOutQ_attributes = {
  .name = "msgOutQ"
};
/* Definitions for keysMutex */
osMutexId_t keysMutexHandle;
const osMutexAttr_t keysMutex_attributes = {
  .name = "keysMutex"
};
/* Definitions for knobsMutex */
osMutexId_t knobsMutexHandle;
const osMutexAttr_t knobsMutex_attributes = {
  .name = "knobsMutex"
};
/* Definitions for read mutex */
osMutexId_t readMutexHandle;
const osMutexAttr_t readMutex_attributes = {
  .name = "readMutex"
};
/* Definitions for CAN_TX_Semaphore */
osSemaphoreId_t CAN_TX_SemaphoreHandle;
const osSemaphoreAttr_t CAN_TX_Semaphore_attributes = {
  .name = "CAN_TX_Semaphore"
};
/* USER CODE BEGIN PV */


uint16_t output_LUT[OUTPUT_SAMPLES];

const uint32_t wavetable_sizes [12] = {
    C_SAMPLES,
    C_SHARP_SAMPLES,
    D_SAMPLES,
    D_SHARP_SAMPLES,
    E_SAMPLES,
    F_SAMPLES,
    F_SHARP_SAMPLES,
    G_SAMPLES,
    G_SHARP_SAMPLES,
    A_SAMPLES,
    A_SHARP_SAMPLES,
    B_SAMPLES,
};

int16_t C_LUT          [C_SAMPLES];
int16_t C_SHARP_LUT    [C_SHARP_SAMPLES];
int16_t D_LUT          [D_SAMPLES];
int16_t D_SHARP_LUT    [D_SHARP_SAMPLES];
int16_t E_LUT          [E_SAMPLES];
int16_t F_LUT          [F_SAMPLES];
int16_t F_SHARP_LUT    [F_SHARP_SAMPLES];
int16_t G_LUT          [G_SAMPLES];
int16_t G_SHARP_LUT    [G_SHARP_SAMPLES];
int16_t A_LUT          [A_SAMPLES];
int16_t A_SHARP_LUT    [A_SHARP_SAMPLES];
int16_t B_LUT          [B_SAMPLES];

int16_t* lookup_tables [12] = {
    C_LUT,
    C_SHARP_LUT,
    D_LUT,
    D_SHARP_LUT,
    E_LUT,
    F_LUT,
    F_SHARP_LUT,
    G_LUT,
    G_SHARP_LUT,
    A_LUT,
    A_SHARP_LUT,
    B_LUT,
};

uint16_t lookup_indices [12];

uint32_t DMAkeys;
uint32_t DMAkeys2;

volatile bool outbits [7] = {1, 1, 1, 1, 1, 1, 1};  

const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

u8g2_t u8g2;
CAN_MSG_t RX;

const float fs = 22000;
const float fA = 440;
float dreal[12];
float dimag[12];

volatile uint16_t keys = 0x0FFF;
volatile uint16_t prev_keys = 0x0FFF;
volatile uint16_t knobs = 0xFF;
volatile uint16_t prev_knobs = 0xFF;

volatile bool HKIW = false;
volatile bool HKIE = false;

uint16_t volume = 8;
uint16_t octave = 3;
uint16_t default_octave = 3;
uint8_t wave_form = 0;

const char* keyNotes[12] = {
  "Do", "Do#",
  "Re", "Re#",
  "Mi",
  "Fa", "Fa#",
  "Sol", "Sol#",
  "La", "La#",
  "Si"
};
char* notesPressed[12] = {'-','-','-','-','-','-','-','-','-','-','-','-'};

//uint8_t TX_Message[8] = {'A', 'L', 'I', 'B', 'E', 'S', 'T', '!'};

const uint32_t IDout = 0x123;
const uint32_t IDin = 0x123;

uint32_t UID0;

// handshaking
bool handshakeRequest = 1;
volatile uint8_t keyboard_count = 1;
volatile uint8_t keyboard_position = 0;

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
void decode(void *argument);
void CAN_Transmit(void *argument);
void handshake(void *argument);

/* USER CODE BEGIN PFP */

void serialPrint(char val[]);
void serialPrintln(char val[]);
void delayMicro(uint16_t us);

void synthesize_waves(int index);

uint32_t setCANFilter(uint32_t filterID, uint32_t maskID, uint32_t filterBank);
uint32_t CAN_TX(uint32_t ID, uint8_t data[8]);
uint32_t CAN_CheckRXLevel();
uint32_t CAN_RX(uint32_t *ID, uint8_t data[8]);

void setOutMuxBit(const uint8_t bitIdx, const bool value);
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr);
uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

void setRow(uint8_t rowIdx);
void setMuxIO();
uint8_t readCols();

/*
uint16_t readKeys();
uint16_t readKnobs();
*/
int16_t changeKnobState(uint8_t knob_state, uint8_t previousKnobState, uint16_t volume, int8_t top_limit, int8_t bottom_limit);
void scanKnob(uint16_t localKnobs, uint16_t prev_Knobs, uint8_t knob_index, char type );
void choose_wave_gen(uint8_t wave, int table, uint32_t samples);
void display_wave(uint16_t x, uint16_t y);

//void rotationSteps(float *dreal, float *dimag);

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

//	rotationSteps(dreal, dimag);

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

	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)output_LUT, OUTPUT_SAMPLES, DAC_ALIGN_12B_R);
//	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

	setOutMuxBit(DRST_BIT, GPIO_PIN_RESET);
	delayMicro(2);
	setOutMuxBit(DRST_BIT, GPIO_PIN_SET);
	u8g2_Setup_ssd1305_i2c_128x32_noname_f(&u8g2, U8G2_R0, u8x8_byte_i2c,
			u8x8_gpio_and_delay);
	u8g2_InitDisplay(&u8g2);
	u8g2_ClearDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);
	setOutMuxBit(DEN_BIT, GPIO_PIN_SET);

	setCANFilter(IDin, 0x7ff, 0);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

	serialPrintln("charIOT-Key-C");
	UID0 = HAL_GetUIDw0();

        //Generate initial wave tables

        char buf [20];
        for (int t = 0; t < 12; t++) {

            uint32_t samples =  wavetable_sizes[t];

            sprintf(buf, "\n\n Lut: %i------", t);
            //serialPrintln(buf);

            choose_wave_gen(wave_form, t, samples); // 0 - sawtooth; 1 - sine; 2 - square; 3 - triangle; 4 - special; other/default - sawtooth
//            for (int i = 0; i < samples; i++) {
//                lookup_tables[t][i] = 2048 * sin(2.0 * PI * (float)i / (float) samples);
//                sprintf(buf, "%i %i ", i, lookup_tables[t][i]);
//                //serialPrintln(buf);
//            }
        }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of keysMutex */
  keysMutexHandle = osMutexNew(&keysMutex_attributes);

  /* creation of knobsMutex */
  knobsMutexHandle = osMutexNew(&knobsMutex_attributes);

  readMutexHandle = osMutexNew(&readMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	osMutexRelease(keysMutexHandle);
	osMutexRelease(knobsMutexHandle);
	osMutexRelease(readMutexHandle);
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of CAN_TX_Semaphore */
  CAN_TX_SemaphoreHandle = osSemaphoreNew(3, 3, &CAN_TX_Semaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	osSemaphoreRelease(CAN_TX_SemaphoreHandle);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of msgInQ */
  msgInQHandle = osMessageQueueNew (36, sizeof(CAN_MSG_t), &msgInQ_attributes);

  /* creation of msgOutQ */
  msgOutQHandle = osMessageQueueNew (36, sizeof(CAN_MSG_t), &msgOutQ_attributes);

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

  /* creation of decodeTask */
  decodeTaskHandle = osThreadNew(decode, NULL, &decodeTask_attributes);

  /* creation of CAN_TX_TaskName */
  CAN_TX_TaskNameHandle = osThreadNew(CAN_Transmit, NULL, &CAN_TX_TaskName_attributes);

  /* creation of handshakeTask */
  handshakeTaskHandle = osThreadNew(handshake, NULL, &handshakeTask_attributes);

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

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
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
	char rn[2] = "\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*) rn, 2, 10);

}

void delayMicro(uint16_t us) {

	htim7.Instance->CNT = 0;
	while (htim7.Instance->CNT < us)
		;

}

inline void synthesize_waves(int index){

    int32_t out = 0;
    uint8_t keys_pressed = 0;
    // Wavetables are generated assuming that the step size is 2.
    int8_t step_octave = 1 << (octave - 1);

    HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
    
    for (int t = 0; t < 12; t++){
        bool pressed = ~DMAkeys & ( 1 << (t));

        if (pressed) {
            lookup_indices[t] = (lookup_indices[t] + (step_octave)) % wavetable_sizes[t];
            keys_pressed += 1;
            out += lookup_tables[t][lookup_indices[t]];
        }
    }

//    for (int t = 0; t < 12; t++){
//        bool pressed = ~DMAkeys2 & ( 1 << (t));
//
//        if (pressed) {
//            lookup_indices[t] = (lookup_indices[t] + (4)) % wavetable_sizes[t];
//            keys_pressed += 1;
//            out += lookup_tables[t][lookup_indices[t]];
//        }
//    }

//    output_LUT[index] = ((uint16_t)(out / (1 + keys_pressed))) + 2048; // VOLUME to be added here
    output_LUT[index] = ((uint16_t)(out >> (12 - volume))) + 2048; // VOLUME to be added here

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

uint32_t setCANFilter(uint32_t filterID, uint32_t maskID, uint32_t filterBank) {

	CAN_FilterTypeDef filterInfo = { 0 };

	filterInfo.FilterIdHigh = (filterID << 5) & 0xffe0;
	filterInfo.FilterIdLow = 0;
	filterInfo.FilterMaskIdHigh = (maskID << 5) & 0xffe0;
	filterInfo.FilterMaskIdLow = 0;
	filterInfo.FilterFIFOAssignment = 0;
	filterInfo.FilterBank = filterBank & 0xf;
	filterInfo.FilterMode = CAN_FILTERMODE_IDMASK;
	filterInfo.FilterScale = CAN_FILTERSCALE_32BIT;
	filterInfo.FilterActivation = CAN_FILTER_ENABLE;
	filterInfo.SlaveStartFilterBank = 0;

	return (uint32_t) HAL_CAN_ConfigFilter(&hcan1, &filterInfo);

}

uint32_t CAN_TX(uint32_t ID, uint8_t data[8]) {

	CAN_TxHeaderTypeDef txHeader = { 0 };

	txHeader.StdId = ID & 0x7ff;
	txHeader.ExtId = 0;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = 8;
	txHeader.TransmitGlobalTime = DISABLE;

	while (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
		;

	return (uint32_t) HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, NULL);

}

uint32_t CAN_CheckRXLevel() {

	return HAL_CAN_GetRxFifoFillLevel(&hcan1, 0);

}

uint32_t CAN_RX(uint32_t *ID, uint8_t data[8]) {

	CAN_RxHeaderTypeDef rxHeader;

	while (!HAL_CAN_GetRxFifoFillLevel(&hcan1, 0))
		;

	uint32_t result = (uint32_t) HAL_CAN_GetRxMessage(&hcan1, 0, &rxHeader,
			data);

	*ID = rxHeader.StdId;

	return result;

}

void setOutMuxBit(const uint8_t bitIdx, const bool value) {

	HAL_GPIO_WritePin(REN_GPIO_Port, REN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RA0_GPIO_Port, RA0_Pin, bitIdx & 0x01);
	HAL_GPIO_WritePin(RA1_GPIO_Port, RA1_Pin, bitIdx & 0x02);
	HAL_GPIO_WritePin(RA2_GPIO_Port, RA2_Pin, bitIdx & 0x04);
	HAL_GPIO_WritePin(OUT_GPIO_Port, OUT_Pin, value);
	HAL_GPIO_WritePin(REN_GPIO_Port, REN_Pin, GPIO_PIN_SET);
	delayMicro(5);
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


void selectRow(uint8_t rowIdx) {
    HAL_GPIO_WritePin(REN_GPIO_Port, REN_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(RA0_GPIO_Port, RA0_Pin, rowIdx & 0x01);
    HAL_GPIO_WritePin(RA1_GPIO_Port, RA1_Pin, rowIdx & 0x02);
    HAL_GPIO_WritePin(RA2_GPIO_Port, RA2_Pin, rowIdx & 0x04);
}


void setMuxIO() {

    uint16_t local_keys = 0;
    uint16_t local_knobs = 0;
    bool local_HKIW = 0;
    bool local_HKIE = 0;


    for(int r = 0; r < 7; r++){
        selectRow(r);
        HAL_GPIO_WritePin(OUT_GPIO_Port, OUT_Pin, outbits[r]);
        HAL_GPIO_WritePin(REN_GPIO_Port,REN_Pin, GPIO_PIN_SET);
        delayMicro(5);
        if( r < 3) {
            local_keys |= readCols() << (r * 4);
        } else if (r < 5) {
            local_knobs |= (readCols() << ((r - 3) * 4));
        } else if (r == 5) {
            local_HKIW = readCols() >> 3;
        } else {
            local_HKIE = readCols() >> 3;
        }
        HAL_GPIO_WritePin(REN_GPIO_Port, REN_Pin, GPIO_PIN_RESET);
    }

    __atomic_store_n(&HKIW, local_HKIW, __ATOMIC_RELAXED);
    __atomic_store_n(&HKIE, local_HKIE, __ATOMIC_RELAXED);
    __atomic_store_n(&keys, local_keys, __ATOMIC_RELAXED);
    __atomic_store_n(&knobs, local_knobs, __ATOMIC_RELAXED);
}

/*
void setRow(uint8_t rowIdx) {

	HAL_GPIO_WritePin(REN_GPIO_Port, REN_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(RA0_GPIO_Port, RA0_Pin, rowIdx & 0x01);
	HAL_GPIO_WritePin(RA1_GPIO_Port, RA1_Pin, rowIdx & 0x02);
	HAL_GPIO_WritePin(RA2_GPIO_Port, RA2_Pin, rowIdx & 0x04);

	HAL_GPIO_WritePin(REN_GPIO_Port, REN_Pin, GPIO_PIN_SET);

}*/

uint8_t readCols() {

	uint8_t C0_val = HAL_GPIO_ReadPin(C0_GPIO_Port, C0_Pin);
	uint8_t C1_val = HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin);
	uint8_t C2_val = HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin);
	uint8_t C3_val = HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin);

	return (C3_val << 3) | (C2_val << 2) | (C1_val << 1) | C0_val;

}

/*
uint16_t readKeys() {

	osMutexAcquire(readMutexHandle, osWaitForever);
	uint16_t keysRead = 0;

	for (int i = 0; i <= 2; i++) {

		//setRow(i);
                setOutMuxBit(i, outbits[i]);
		delayMicro(5);
		keysRead |= readCols() << (4 * i);

	}
	osMutexRelease(readMutexHandle);
	return keysRead;

}

uint16_t readKnobs() {

	osMutexAcquire(readMutexHandle, osWaitForever);
	uint16_t knobsRead = 0;

	for (int i = 3; i <= 4; i++) {

                setOutMuxBit(i, outbits[i]);
		delayMicro(5);
		knobsRead |= readCols() << (4 * (i-3));

	}

	for (int i = 5; i < 7; i++) {
                setOutMuxBit(i, outbits[i]);
		delayMicro(5);
	}

	osMutexRelease(readMutexHandle);

	return knobsRead;

}*/

int16_t changeKnobState(uint8_t knob_state, uint8_t previousKnobState, uint16_t knobRotation, int8_t top_limit, int8_t bottom_limit){
	int16_t rotation = 0;
	int current_knob = knob_state;
	int prev_knob = previousKnobState;

	// upper and bottom levels for knob
	if ((((prev_knob == 0b11) && (current_knob == 0b10)) ||
	  ((prev_knob == 0b00) && (current_knob == 0b01))) &&
		knobRotation < top_limit
	){
	rotation ++;
	} else
	if ((((prev_knob == 0b01) && (current_knob == 0b00)) ||
	   ((prev_knob == 0b10) && (current_knob == 0b11))) &&
		 knobRotation > bottom_limit
	) {
	rotation --;
	}

	return rotation;
}

void scanKnob(uint16_t localKnobs, uint16_t prevKnobs, uint8_t knob_index, char type ) {
	uint8_t shift_row = (knob_index >= 2) ? 0 : 4;
	uint8_t row = 0xF;
	uint8_t knob_on_row = 1 - knob_index % 2;

	uint8_t rowKnobStates 	  = (localKnobs 	 >> shift_row) & row;
	uint8_t rowPrevKnobStates = (prevKnobs >> shift_row) & row;

//	char s[32];
//	sprintf(s, "rowKnobStates:%x", rowKnobStates);
//	serialPrintln(s);

	uint8_t knobState		  = (rowKnobStates 	   >> knob_on_row*2) & 0b11;
	uint8_t previousKnobState = (rowPrevKnobStates >> knob_on_row*2) & 0b11;


//	UPDATE GLOBAL VARIABLES
	osMutexAcquire(knobsMutexHandle, osWaitForever);
	__atomic_store_n(&knobs, localKnobs, __ATOMIC_RELAXED);
	osMutexRelease(knobsMutexHandle);

	if (previousKnobState != knobState) {
		osMutexAcquire(knobsMutexHandle, osWaitForever);
		__atomic_store_n(&prev_knobs, localKnobs, __ATOMIC_RELAXED);
		osMutexRelease(knobsMutexHandle);

		if (type == 'v'){
			int16_t change_volume = changeKnobState(knobState, previousKnobState, volume, 12, 0);
			volume = volume + change_volume;
		//	sprintf(s, "volume: %d", volume);
		//	serialPrintln(s);
		} else if (type == 'o'){
			int16_t change_octave = changeKnobState(knobState, previousKnobState, octave, 8, 2); // can only go one lower than the default octave
			octave = octave + change_octave;
		//	sprintf(s, "octave: %d", octave);
		//	serialPrintln(s);
		} else if (type == 'w'){
			int16_t change_wave = changeKnobState(knobState, previousKnobState, wave_form, 4, 0);
			wave_form = wave_form + change_wave;
		//	sprintf(s, "wave_form: %d", wave_form);
		//	serialPrintln(s);
			for (int t = 0; t < 12; t++) {
//			TODO: TIMING PROBLEM - might take too long to generate lookup tables at every change of the wave form knob
				uint32_t samples =  wavetable_sizes[t];
				choose_wave_gen(wave_form, t, samples); // 0 - sawtooth; 1 - sine; 2 - square; 3 - triangle; 4 - special; other/default - sawtooth
			}
		}
	}

}

void choose_wave_gen(uint8_t wave, int t, uint32_t samples){
	if(wave == 0){
		//      SAWTOOTH WAVES
		int half_samples = samples / 2;
		for (int i = 0; i < samples; i++) {
			lookup_tables[t][i] = (i <= half_samples) ? 2048 * ((float)(i-half_samples) / (float) samples) : 2048 * ((float)(i-half_samples) / (float) samples);
		}
	} else if(wave == 1){
		//      PURE SINE WAVES
		for (int i = 0; i < samples; i++) {
			lookup_tables[t][i] = 2048 * sin(2.0 * PI * (float)i / (float) samples);
		}
	} else if(wave == 2){
		//      SQUARE WAVES
		int half_samples = samples / 2;
		for (int i = 0; i < samples; i++) {
			lookup_tables[t][i] = (i <= half_samples) ? 2048 * (1.0) : 2048 * (-1.0);
		}
	} else if(wave == 3){
		//      TRIANGLE WAVES
		int half_samples = samples / 2;
		int first_fourth = samples / 4;
		int third_fourth = half_samples + first_fourth;
		for (int i = 0; i < samples; i++) {
			lookup_tables[t][i] = (i <= first_fourth) ?
									2048 * ((float)(-i) / (float) samples)
								:
									(i <= third_fourth) ? 2048 * ((float)(i-half_samples) / (float) samples) : 2048 * ((float)(samples-i) / (float) samples);
		}
	} else if(wave == 4){
		////    Special WAVE
		for (int i = 0; i < samples; i++) {
			float harmonic_sample =  sin(2.0 * PI * (float)i / ((float) samples));
			harmonic_sample += sin(2.0 * PI * (float)i * 3 / ((float) samples)) / 3;
			harmonic_sample += sin(2.0 * PI * (float)i * 7 / ((float) samples)) / 7;
			harmonic_sample += sin(2.0 * PI * (float)i * 8 / ((float) samples)) / 8;
			lookup_tables[t][i] = (2048 * harmonic_sample);
		}
	} else {
		//      SAWTOOTH WAVES
		int half_samples = samples / 2;
		for (int i = 0; i < samples; i++) {
			lookup_tables[t][i] = (i <= half_samples) ? 2048 * ((float)(i-half_samples) / (float) samples) : 2048 * ((float)(i-half_samples) / (float) samples);
		}
	}
}

void display_wave(uint16_t x, uint16_t y){
	switch(wave_form){
		case 0: //SAWTOOTH
			u8g2_SetFont(&u8g2, u8g2_font_7x13_t_symbols);
			u8g2_DrawUTF8(&u8g2, x, y, "//");
			break;
		case 1: //SINE
			u8g2_SetFont(&u8g2, u8g2_font_7x13_t_symbols);
			u8g2_DrawUTF8(&u8g2, x, y, "◠◡");
			break;
		case 2: //SQUARE
			u8g2_SetFont(&u8g2, u8g2_font_7x13_t_symbols);
			u8g2_DrawUTF8(&u8g2, x, y, " \u25a0");
			break;
		case 3: //TRIANGLE
			u8g2_SetFont(&u8g2, u8g2_font_7x13_t_symbols);
			u8g2_DrawUTF8(&u8g2, x, y, " \u25b2");
			break;
		case 4: //special
			u8g2_SetFont(&u8g2, u8g2_font_7x13_t_symbols);
			u8g2_DrawUTF8(&u8g2, x, y, " \u265b");
			break;
		default://SAWTOOTH
			u8g2_SetFont(&u8g2, u8g2_font_7x13_t_symbols);
			u8g2_DrawUTF8(&u8g2, x, y, "//");
			break;
	}

}

//void rotationSteps(float *dreal, float *dimag) {
//
//	float phi;
//
//	for (int i = 0; i < 12; i++) {
//
//		phi = 2 * M_PI * fA * pow(2, (i - 9) / 12.0) / fs;
//		dreal[i] = cos(phi);
//		dimag[i] = sin(phi);
//
//	}
//
//}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	CAN_MSG_t RX;
	CAN_RX(&RX.ID, RX.Message);
	osMessageQueuePut(msgInQHandle, &RX.Message, 0, 0);

}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {

	osSemaphoreRelease(CAN_TX_SemaphoreHandle);

}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {

	osSemaphoreRelease(CAN_TX_SemaphoreHandle);

}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {

	osSemaphoreRelease(CAN_TX_SemaphoreHandle);

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
		osDelay(100);
                //serialPrintln("default task");
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


                setMuxIO();
		uint16_t localKeys = __atomic_load_n(&keys, __ATOMIC_RELAXED);
		uint16_t localKnobs = __atomic_load_n(&knobs, __ATOMIC_RELAXED);

//		char key_s[16];
//		sprintf(key_s, "%x", localKeys);
//		char knobs_s[16];
//		sprintf(knobs_s, "%x", localKnobs);
//
//		serialPrint("keys: ");
//		serialPrintln(key_s);
//		serialPrint("knobs: ");
//		serialPrintln(knobs_s);
		uint8_t keys_pressed = 0;
		for (int t = 0; t < 12; t++){
			bool pressed = ~localKeys & ( 1 << (t));

			if (pressed) {
				notesPressed[t] = keyNotes[t];
				keys_pressed += 1;
			} else {
				notesPressed[t] = '-';
			}
		}

		scanKnob(localKnobs, (uint16_t) prev_knobs, 3, 'v');
		scanKnob(localKnobs, (uint16_t) prev_knobs, 2, 'o');
		scanKnob(localKnobs, (uint16_t) prev_knobs, 1, 'w');

//		---------------------------- SEND OVER CAN
		CAN_MSG_t TX;
		TX.ID = IDout;
                /*
		char *msg = "AliBest!"; // just remember to stick to size of TX.Message, if it is bigger, it gets cut off
		for (int i = 0; i < sizeof(TX.Message); i++){
			TX.Message[i] = msg[i];
		}*/

                TX.Message[0] = localKeys & 0x0FF;
                TX.Message[1] = localKeys >> 8;

		osMessageQueuePut(msgOutQHandle, &TX, 0, 0);

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
		osMutexRelease(knobsMutexHandle);

		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_new3x9pixelfont_tr);

//		PRINTING THE NOTES PRESSED
		uint8_t string_size = 2;
		uint8_t space = 3;
		char o_s[16];
		sprintf(o_s, "|%x|", octave);
		u8g2_DrawStr(&u8g2, string_size, 7, o_s);
		string_size += 10;
		for (int t = 0; t < 12; t++){
			if (notesPressed[t] != '-') {
				uint8_t w = u8g2_GetStrWidth(&u8g2, keyNotes[t]);
				u8g2_DrawStr(&u8g2, string_size, 7, notesPressed[t]);
				string_size += w + space;
			}
		}
//                uint32_t localDMAkeys2 = __atomic_load_n(&DMAkeys2, __ATOMIC_RELAXED);

                char buf[20];
                sprintf(buf, "%x", RX.Message[1]);
                //serialPrintln(buf);

//		PRINTING VOLUME
		u8g2_DrawButtonUTF8(&u8g2, 105, 30, U8G2_BTN_BW1, 18,  4,  2, "Vol:");
		char volume_s[16];
		sprintf(volume_s, "%x", volume);
		u8g2_DrawStr(&u8g2, 118, 30, volume_s);

//		PRINTING Octave
		u8g2_DrawButtonUTF8(&u8g2, 75, 30, U8G2_BTN_BW1, 18,  4,  2, "Oct:");
		char s[16];
		sprintf(s, "%x", octave);
		u8g2_DrawStr(&u8g2, 89, 30, s);

//		PRINTING WAVE_FORM
		u8g2_DrawButtonUTF8(&u8g2, 33, 30, 0, 30,  4,  3, "Wave:");
		char wave_s[16];
		sprintf(wave_s, "%x", wave_form);
//		u8g2_DrawStr(&u8g2, 61, 30, wave_s);
		display_wave(51, 30);

//		PRINTING PET
		u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
		if (localKeys == 0x0FFF) {

//			u8g2_DrawStr(&u8g2, 70, 10, "- ^_^ -");
			u8g2_SetFont(&u8g2, u8g2_font_streamline_all_t);
			u8g2_DrawUTF8(&u8g2, 2, 30, " \u029a");

		} else {

//			u8g2_DrawStr(&u8g2, 70, 10, "- ^0^ -");
			u8g2_SetFont(&u8g2, u8g2_font_streamline_all_t); //21x21
			u8g2_DrawUTF8(&u8g2, 2, 30, " \u0299");
			u8g2_SetFont(&u8g2, u8g2_font_unifont_t_0_76); //16x16
			u8g2_DrawUTF8(&u8g2, 16, 27, " \u266a");
			u8g2_DrawUTF8(&u8g2, 13, 19, " \u266a");

		}

		u8g2_SendBuffer(&u8g2);

	}

  /* USER CODE END displayUpdateTask */
}

/* USER CODE BEGIN Header_decode */
/**
 * @brief Function implementing the decodeTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_decode */
void decode(void *argument)
{
  /* USER CODE BEGIN decode */
	CAN_MSG_t RX;

	/* Infinite loop */
	for (;;) {

		osMessageQueueGet(msgInQHandle, &RX, NULL, osWaitForever);

		if (RX.Message[0] == 'H') {

			keyboard_count += 1;

			// termination (UNTESTED)

			if (RX.Message[1] == 'X') {

				handshakeRequest = 0;	// or osEventFlagsClear

				// write the signals again to check for future disconnections
                                //
                                
                                osMutexAcquire(readMutexHandle, osWaitForever);
				setOutMuxBit(HKOE_BIT, GPIO_PIN_SET);
				setOutMuxBit(HKOE_BIT, GPIO_PIN_SET);
                                osMutexRelease(readMutexHandle);

				// HERE, insert code to (re)start everything else
			}

		}

//		char hexID[3];
//
//		sprintf(hexID, "%lX", RX.ID);
//
//		u8g2_ClearBuffer(&u8g2);
//		u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
//		u8g2_DrawStr(&u8g2, 2, 10, hexID);
//		u8g2_DrawStr(&u8g2, 2, 20, (char*) RX.Message);
//		u8g2_SendBuffer(&u8g2);

	}
  /* USER CODE END decode */
}

/* USER CODE BEGIN Header_CAN_Transmit */
/**
 * @brief Function implementing the CAN_TX_TaskName thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CAN_Transmit */
void CAN_Transmit(void *argument)
{
  /* USER CODE BEGIN CAN_Transmit */
	/* Infinite loop */

	CAN_MSG_t TX;

	for (;;) {

		osMessageQueueGet(msgOutQHandle, &TX, NULL, osWaitForever);
		osSemaphoreAcquire(CAN_TX_SemaphoreHandle, osWaitForever);
		CAN_TX(TX.ID, TX.Message);

	}

  /* USER CODE END CAN_Transmit */
}

/* USER CODE BEGIN Header_handshake */
/**
 * @brief Function implementing the handshakeTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_handshake */
void handshake(void *argument)
{
  /* USER CODE BEGIN handshake */

	if (handshakeRequest) {	// this could replaced with a flag ?
            // maybe something like osEventFlagsWait ?
		// osEventFlagsSet could be called from the task reading user inputs

		// write the outgoing handshaking signals to high
		
                /*osMutexAcquire(readMutexHandle, osWaitForever);
                setOutMuxBit(HKOW_BIT, GPIO_PIN_SET);
		setOutMuxBit(HKOE_BIT, GPIO_PIN_SET);
                osMutexRelease(readMutexHandle);*/
                outbits[5] = 1;
                outbits[6] = 1;

    		osDelay(2000);


		// read the east-side incoming handshaking signal
		// the signal is inverted, ie, 0 if there is a keyboard attached
		// hence, here, only the last keyboard will read high
		// this is used to terminate the handshaking process
		

		// the keyboards turn off their east outgoing signal in turn
		// starting from the leftmost keyboard

		// wait for the west-side handshaking signal to go high
		while (!HKIW) {
			osDelay(100);
		}

		// keyboard_count is incremented at every received CAN message
		// -> see the decode task

		keyboard_position = keyboard_count - 1;
                octave = keyboard_position + 3;

		// inform other keyboards
		// send unique ID and position as per instructions

		CAN_MSG_t TX;

		TX.ID = IDout;
		TX.Message[0] = 'H';
		TX.Message[1] = (uint8_t) (UID0 & 0xF000) >> 24;
		TX.Message[2] = (uint8_t) (UID0 & 0x0F00) >> 16;
		TX.Message[3] = (uint8_t) (UID0 & 0x00F0) >> 8;
		TX.Message[4] = (uint8_t) (UID0 & 0x000F);
		TX.Message[5] = keyboard_position;

		osMessageQueuePut(msgOutQHandle, &TX, 0, 0);

		// display the data to check

		char UID0text[8];
		sprintf(UID0text, "%lX", UID0);
                serialPrintln(UID0text);

		char posText[2];
		sprintf(posText, "%i", keyboard_position);
                serialPrintln(posText);

                /*
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
		u8g2_DrawStr(&u8g2, 2, 10, UID0text);
		u8g2_DrawStr(&u8g2, 2, 20, posText);
		u8g2_SendBuffer(&u8g2);
                */

		HAL_Delay(100);

		// turn off the east outgoing signal to inform the next keyboard

                //setOutMuxBit(HKOE_BIT, GPIO_PIN_RESET);
                outbits[6] = 0;
	}


	/* Infinite loop */
    for(;;){
        osDelay(1);

    }
  /* USER CODE END handshake */
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
