#include "main.h"

// Device handles
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac_ch1;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart2;

// Device configuration functions
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_USART2_UART_Init(void);
void MX_CAN1_Init(void);
void MX_I2C1_Init(void);
void MX_ADC1_Init(void);
void MX_DAC1_Init(void);
void MX_TIM6_Init(void);
void MX_TIM7_Init(void);
void MX_TIM2_Init(void);
