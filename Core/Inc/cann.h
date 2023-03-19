#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include <main.h>
#include "cmsis_os.h"

typedef struct {
	uint8_t Message[8];
	uint32_t ID;
} CAN_MSG_t;

typedef enum {
    DEN_BIT = 3,
    DRST_BIT = 4,
    HKOW_BIT = 5,
    HKOE_BIT = 6,
} DFFBitFunction;

void decode(void *argument);
void CAN_Transmit(void *argument);
void handshake(void *argument);
uint32_t setCANFilter(uint32_t filterID, uint32_t maskID, uint32_t filterBank);
uint32_t CAN_TX(uint32_t ID, uint8_t data[8]);
uint32_t CAN_CheckRXLevel();
uint32_t CAN_RX(uint32_t *ID, uint8_t data[8]);
extern void setOutMuxBit(const uint8_t bitIdx, const bool value);

// Device handles
extern CAN_HandleTypeDef hcan1;

// Transmit semaphore
extern osSemaphoreId_t CAN_TX_SemaphoreHandle;

// CAN Message queues
extern osMessageQueueId_t msgInQHandle;
extern osMessageQueueId_t msgOutQHandle;

extern osMutexId_t readMutexHandle;

extern uint32_t UID0;
extern bool handshakeRequest;

extern const uint32_t IDin;

extern volatile bool outbits [7];

extern volatile bool HKIW;
extern volatile bool HKIE;

extern uint16_t octave;
extern bool selected;
extern bool controller;

extern uint16_t allKeys[10];
