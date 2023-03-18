#include "main.h"
#include "wavegen.h"
#include "cmsis_os.h"
#include "hardware_config.h"
#include "cann.h"

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

osMutexId_t readMutexHandle;

volatile bool outbits [7];

uint16_t octave;

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

uint32_t DMAkeys;
uint32_t DMAkeys2;

volatile bool outbits [7] = {1, 1, 1, 1, 1, 1, 1};  

u8g2_t u8g2;
CAN_MSG_t RX;

volatile uint16_t keys = 0x0FFF;
volatile uint16_t prev_keys = 0x0FFF;
volatile uint16_t knobs = 0xFF;
volatile uint16_t prev_knobs = 0xFF;

uint16_t volume = 3;
uint16_t octave = 3;
uint16_t default_octave = 3;

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

void StartDefaultTask(void *argument);
void scanKeysTask(void *argument);
void displayUpdateTask(void *argument);

void serialPrint(char val[]);
void serialPrintln(char val[]);
void delayMicro(uint16_t us);

void setOutMuxBit(const uint8_t bitIdx, const bool value);
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr);
uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

void setMuxIO();
uint8_t readCols();

int16_t changeKnobState(uint8_t knob_state, uint8_t previousKnobState, uint16_t volume, int8_t top_limit, int8_t bottom_limit);
void scanKnob(uint16_t localKnobs, uint16_t prev_Knobs, uint8_t knob_index, char type );

int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();
    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
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

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim7);
    HAL_TIM_Base_Start_IT(&htim6);

    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)output_wave, OUTPUT_SAMPLES, DAC_ALIGN_12B_R);
    //	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    //	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

    setOutMuxBit(DRST_BIT, GPIO_PIN_RESET);
    delayMicro(2);
    setOutMuxBit(DRST_BIT, GPIO_PIN_SET);
    u8g2_Setup_ssd1305_i2c_128x32_noname_f(&u8g2, U8G2_R0, u8x8_byte_i2c, u8x8_gpio_and_delay);
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

    init_lookup_tables();
    set_output_waveform(CLARINET);

    // Init scheduler
    osKernelInitialize();

    // Mutex creation
    keysMutexHandle = osMutexNew(&keysMutex_attributes);
    knobsMutexHandle = osMutexNew(&knobsMutex_attributes);
    readMutexHandle = osMutexNew(&readMutex_attributes);

    // Add mutexes
    osMutexRelease(keysMutexHandle);
    osMutexRelease(knobsMutexHandle);
    osMutexRelease(readMutexHandle);

    // Create semaphores
    const osSemaphoreAttr_t CAN_TX_Semaphore_attributes = {.name = "CAN_TX_Semaphore"};
    CAN_TX_SemaphoreHandle = osSemaphoreNew(3, 3, &CAN_TX_Semaphore_attributes);

    // Add semaphores
    osSemaphoreRelease(CAN_TX_SemaphoreHandle);

    // Create queues
    const osMessageQueueAttr_t msgInQ_attributes = {.name = "msgInQ"};
    const osMessageQueueAttr_t msgOutQ_attributes = {.name = "msgOutQ"};
    msgInQHandle = osMessageQueueNew (36, sizeof(CAN_MSG_t), &msgInQ_attributes);
    msgOutQHandle = osMessageQueueNew (36, sizeof(CAN_MSG_t), &msgOutQ_attributes);

    // Create threads
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    scanKeysHandle = osThreadNew(scanKeysTask, NULL, &scanKeys_attributes);
    displayUpdateHandle = osThreadNew(displayUpdateTask, NULL, &displayUpdate_attributes);
    decodeTaskHandle = osThreadNew(decode, NULL, &decodeTask_attributes);
    CAN_TX_TaskNameHandle = osThreadNew(CAN_Transmit, NULL, &CAN_TX_TaskName_attributes);
    handshakeTaskHandle = osThreadNew(handshake, NULL, &handshakeTask_attributes);

    // Start scheduler
    osKernelStart();
}



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



void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac){

    uint16_t localKeys = __atomic_load_n(&keys, __ATOMIC_RELAXED);

    synthesize_output(localKeys, volume, octave, true);
    
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    
    uint16_t localKeys = __atomic_load_n(&keys, __ATOMIC_RELAXED);

    synthesize_output(localKeys, volume, octave, false);
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

uint8_t readCols() {

	uint8_t C0_val = HAL_GPIO_ReadPin(C0_GPIO_Port, C0_Pin);
	uint8_t C1_val = HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin);
	uint8_t C2_val = HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin);
	uint8_t C3_val = HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin);

	return (C3_val << 3) | (C2_val << 2) | (C1_val << 1) | C0_val;

}


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
			int16_t change_wave = changeKnobState(knobState, previousKnobState, output_wavetype, 4, 0);
			WaveType new_wavetype = (output_wavetype + change_wave) % END_WAVETYPE;
                        //	sprintf(s, "wave_form: %d", wave_form);
                        //	serialPrintln(s);
                        //
                        set_output_waveform(new_wavetype);
		}
	}

}


void StartDefaultTask(void *argument)
{
    for (;;) {
        osDelay(100);
        char buf[20];
        sprintf(buf, "type: %i", output_wavetype);
        //serialPrintln(buf);
    }
}

void scanKeysTask(void *argument)
{
    const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* Infinite loop */
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        setMuxIO();
        uint16_t localKeys = __atomic_load_n(&keys, __ATOMIC_RELAXED);
        uint16_t localKnobs = __atomic_load_n(&knobs, __ATOMIC_RELAXED);

        //char key_s[16];
        //sprintf(key_s, "%x", localKeys);
        //char knobs_s[16];
        //sprintf(knobs_s, "%x", localKnobs);
        //
        //serialPrint("keys: ");
        //serialPrintln(key_s);
        //serialPrint("knobs: ");
        //serialPrintln(knobs_s);
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
    }
}

void displayUpdateTask(void *argument)
{

    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        osMutexAcquire(keysMutexHandle, osWaitForever);

        uint16_t localKeys = __atomic_load_n(&keys, __ATOMIC_RELAXED);

        osMutexRelease(keysMutexHandle);
        osMutexRelease(knobsMutexHandle);

        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_new3x9pixelfont_tr);

        //PRINTING THE NOTES PRESSED
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
        //uint32_t localDMAkeys2 = __atomic_load_n(&DMAkeys2, __ATOMIC_RELAXED);

        char buf[20];
        sprintf(buf, "%x", RX.Message[1]);
        //serialPrintln(buf);

        //PRINTING VOLUME
        u8g2_DrawButtonUTF8(&u8g2, 105, 30, U8G2_BTN_BW1, 18,  4,  2, "Vol:");
        char volume_s[16];
        sprintf(volume_s, "%x", volume);
        u8g2_DrawStr(&u8g2, 118, 30, volume_s);

        //PRINTING Octave
        u8g2_DrawButtonUTF8(&u8g2, 75, 30, U8G2_BTN_BW1, 18,  4,  2, "Oct:");
        char s[16];
        sprintf(s, "%x", octave);
        u8g2_DrawStr(&u8g2, 89, 30, s);

        //PRINTING WAVE_FORM
        u8g2_DrawButtonUTF8(&u8g2, 33, 30, 0, 30,  4,  3, "Wave:");
        char wave_s[16];
        sprintf(wave_s, "%x", output_wavetype);
        //u8g2_DrawStr(&u8g2, 61, 30, wave_s);
        display_wave(&u8g2, 51, 30);

        //PRINTING PET
        u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
        if (localKeys == 0x0FFF) {

                //u8g2_DrawStr(&u8g2, 70, 10, "- ^_^ -");
                u8g2_SetFont(&u8g2, u8g2_font_streamline_all_t);
                u8g2_DrawUTF8(&u8g2, 2, 30, " \u029a");

        } else {
                //u8g2_DrawStr(&u8g2, 70, 10, "- ^0^ -");
                u8g2_SetFont(&u8g2, u8g2_font_streamline_all_t); //21x21
                u8g2_DrawUTF8(&u8g2, 2, 30, " \u0299");
                u8g2_SetFont(&u8g2, u8g2_font_unifont_t_0_76); //16x16
                u8g2_DrawUTF8(&u8g2, 16, 27, " \u266a");
                u8g2_DrawUTF8(&u8g2, 13, 19, " \u266a");
        }

        u8g2_SendBuffer(&u8g2);

    }
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
	if (htim->Instance == TIM16) {
		HAL_IncTick();
	}
}

//This function is executed in case of error occurrence.
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
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
