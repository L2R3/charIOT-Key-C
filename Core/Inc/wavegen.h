#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "csrc/u8g2.h"

#include <main.h>

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

#define TOTAL_LUT_SAMPLES C_SAMPLES + C_SHARP_SAMPLES + D_SAMPLES + D_SHARP_SAMPLES+ E_SAMPLES + F_SAMPLES + F_SHARP_SAMPLES + G_SAMPLES + G_SHARP_SAMPLES + A_SAMPLES + A_SHARP_SAMPLES + B_SAMPLES

#define OUTPUT_SAMPLES C_SAMPLES 

typedef struct {
    int16_t table_data [TOTAL_LUT_SAMPLES];

    // Each lookup table has 12 separate tables - one for each of the twelve frequencies 
    int16_t* freq_tables [12];
} LookupTable;

typedef enum {
    SAWTOOTH = 0,
    SINE,
    SQUARE,
    TRIANGLE,
    CLARINET,
    END_WAVETYPE,
} WaveType;

void generate_waveform(LookupTable* dest, WaveType wave);
void init_lookup_tables();
void init_lookup_table(LookupTable* table);
void synthesize_output(uint16_t keys, uint8_t volume, uint8_t octave, bool first_half);
void set_output_waveform(WaveType wave);
void display_wave(u8g2_t* u8g2, uint16_t x, uint16_t y);
void synthesize_waves(int index);

extern uint16_t output_wave[OUTPUT_SAMPLES];
extern WaveType output_wavetype;
extern LookupTable output_LUT;

