#include "main.h"
#include "csrc/u8g2.h"
// Device handles
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac_ch1;
extern DMA_HandleTypeDef hdma_dac_ch2;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart2;

extern DMA_HandleTypeDef hdma_memtomem_dma1_channel1;

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

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
