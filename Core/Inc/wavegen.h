#include <stdint.h>
#include <math.h>

#include "cmsis_os.h"
#include "csrc/u8g2.h"

#include "hardware_config.h"
#include "main.h"

#ifndef WAVEGEN_H
#define WAVEGEN_H

#define DDS_LUT_SAMPLES 1024
#define DDS_OUT_SAMPLES 4096

#define MAX_KEYBOARDS 8

typedef enum {
    SAWTOOTH = 0,
    SINE,
    SQUARE,
    TRIANGLE,
    CLARINET,
    RETRO1,
    RETRO2,
    END_WAVETYPE,
} WaveType;

void init_lookup_tables();
void generate_waveform(int16_t lookup_table[DDS_LUT_SAMPLES], WaveType wave);
void set_output_waveform(WaveType wave);
void display_wave(u8g2_t* u8g2, uint16_t x, uint16_t y);
void synthesise_output1();
void synthesise_output2();

extern volatile WaveType output_wavetype;
extern uint16_t DDS_OUT[DDS_OUT_SAMPLES];
extern volatile uint16_t allKeys[MAX_KEYBOARDS];
extern volatile uint8_t keyboard_count;
extern volatile int8_t pos_oct_diff;
extern volatile uint8_t volume;

extern osMutexId_t notesMutexHandle;

#endif
