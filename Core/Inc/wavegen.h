#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "csrc/u8g2.h"

#include <main.h>

#ifndef WAVEGEN_H
#define WAVEGEN_H

#define DDS_LUT_SAMPLES 1024
#define DDS_OUT_SAMPLES 8192
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
void synthesize_output(uint16_t keys, uint8_t volume, uint8_t octave, bool first_half);
void set_output_waveform(WaveType wave);
void display_wave(u8g2_t* u8g2, uint16_t x, uint16_t y);
void synthesize_waves(int index);

extern WaveType output_wavetype;
extern uint16_t DDS_OUT[DDS_OUT_SAMPLES];
extern uint16_t allKeys[MAX_KEYBOARDS];
extern volatile uint8_t keyboard_count;
extern volatile uint8_t keyboard_position;
extern osEventFlagsId_t outputFlagHandle;

#endif
