#include "cann.h"

osSemaphoreId_t CAN_TX_SemaphoreHandle;

osMessageQueueId_t msgInQHandle;
osMessageQueueId_t msgOutQHandle;

uint32_t UID0;
bool handshakeRequest = 1;
volatile uint8_t keyboard_count = 1;

const uint32_t IDout = 0x123;
const uint32_t IDin = 0x123;

volatile bool HKIW = false;
volatile bool HKIE = false;

void decode(void *argument) {
	CanMsg_t RX;

	for (;;) {
		osMessageQueueGet(msgInQHandle, &RX, NULL, osWaitForever);

		switch (RX.Message[0]) {
		case HANDSHAKE: {
			keyboard_count += 1;

			// termination (UNTESTED)
			if (RX.Message[1] == TERMINATE) {

				handshakeRequest = 0; // or osEventFlagsClear
				// write the signals again to check for future disconnections

				//                osMutexAcquire(readMutexHandle, osWaitForever);
				//                setOutMuxBit(HKOE_BIT, GPIO_PIN_SET);
				//                setOutMuxBit(HKOE_BIT, GPIO_PIN_SET);
				//                osMutexRelease(readMutexHandle);

				outbits[5] = 1;
				outbits[6] = 1;

				// HERE, insert code to (re)start everything else
			}
		}
			break;
		case OCTAVE_CHANGE: {
			octave = RX.Message[2] + keyboard_position - RX.Message[1];
			int8_t localDiff = keyboard_position - octave;
			__atomic_store_n(&pos_oct_diff, localDiff, __ATOMIC_RELAXED);
		}
			break;
		case KEYS: {
			uint16_t localKeys = (uint16_t) RX.Message[2] << 8 | RX.Message[1];
			allKeys[RX.Message[3]] = localKeys;
		}
			break;
		}
	}
}

void CAN_Transmit(void *argument) {
	CanMsg_t TX;
	for (;;) {
		osMessageQueueGet(msgOutQHandle, &TX, NULL, osWaitForever);
		osSemaphoreAcquire(CAN_TX_SemaphoreHandle, osWaitForever);
		CAN_TX(TX.ID, TX.Message);
	}
}

void handshake(void *argument) {

	// write the outgoing handshaking signals to high
	outbits[5] = 1;
	outbits[6] = 1;

	// wait until other modules turn on
	osDelay(1000);

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
	uint8_t localCount = __atomic_load_n(&keyboard_count, __ATOMIC_RELAXED);
	uint8_t localPosition = keyboard_count - 1;
	__atomic_store_n(&keyboard_position, localPosition, __ATOMIC_RELAXED);
	uint8_t localOctave = keyboard_position + 4;
	__atomic_store_n(&octave, localOctave, __ATOMIC_RELAXED);
	int8_t localDiff = keyboard_position - octave;
	__atomic_store_n(&pos_oct_diff, localDiff, __ATOMIC_RELAXED);

	// inform other keyboards
	// send unique ID and position as per instructions

	CanMsg_t TX;

	TX.ID = IDout;
	TX.Message[0] = 'H';
	TX.Message[1] = (uint8_t) (UID0 & 0xF000) >> 24;
	TX.Message[2] = (uint8_t) (UID0 & 0x0F00) >> 16;
	TX.Message[3] = (uint8_t) (UID0 & 0x00F0) >> 8;
	TX.Message[4] = (uint8_t) (UID0 & 0x000F);
	TX.Message[5] = localPosition;

	osMessageQueuePut(msgOutQHandle, &TX, 0, 0);

	osDelay(100);

	// turn off the east outgoing signal to inform the next keyboard
	outbits[6] = 0;

	const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	for (;;) {

		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		if (selected) {

			is_receiver = !is_receiver;

			vTaskDelay(500);

		}

	}

}

//---------------------
// CAN LIBRARY PROVIDED
//---------------------

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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CanMsg_t RX;
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
