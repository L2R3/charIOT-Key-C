#include "main.h"
#include <stddef.h>
#include <wavegen.h>

uint16_t output_wave[OUTPUT_SAMPLES];

const uint32_t LUT_freq_sizes [12] = {
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

LookupTable output_LUT;
uint16_t output_LUT_indices [12];
WaveType output_wavetype = END_WAVETYPE;

// Lookup tables for the different types of wave that can be produced
// e.g. sine, sawtooth, clarinet etc.
LookupTable waveform_LUTs [END_WAVETYPE];

// Initialize the contents of the all the LUTs
// This includes generating the different instrument
// waveforms as well as initializing each LUT
// so that freq_tables has pointers pointing to the correct
// parts of their data.
void init_lookup_tables(){
    init_lookup_table(&output_LUT);

    for(WaveType type = 0; type < END_WAVETYPE; type++){
        init_lookup_table(&waveform_LUTs[type]);
        generate_waveform(&waveform_LUTs[type], type);
    }

}

void init_lookup_table(LookupTable* table) {
    int32_t offset = 0;
    for(int t = 0; t < 12; t++){
        table->freq_tables[t] = table->table_data + offset;
        offset += LUT_freq_sizes[t];
    }
}

void generate_waveform(LookupTable* dest, WaveType wave){
    for (int t = 0; t < 12; t++) {
        uint32_t samples =  LUT_freq_sizes[t];

        int half_samples = samples / 2;

        switch (wave) {
            case SAWTOOTH: {
                serialPrint("sawtoothgen\n");
                for (int i = 0; i < samples; i++) {
                        dest->freq_tables[t][i] = (i <= half_samples) ? 2048 * ((float)(i-half_samples) / (float) samples) : 2048 * ((float)(i-half_samples) / (float) samples);
                }
            }
            break;
            case SINE: {
                serialPrint("sinegen\n");
                for (int i = 0; i < samples; i++) {
                        dest->freq_tables[t][i] = 2048 * sin(2.0 * PI * (float)i / (float) samples);
                }
            }
            break;
            case SQUARE: {
                serialPrint("squaregen\n");
                for (int i = 0; i < samples; i++) {
                        dest->freq_tables[t][i] = (i <= half_samples) ? 2048 * (1.0) : 2048 * (-1.0);
                }
            }
            break;
            case TRIANGLE: {
                serialPrint("trianglegen\n");
                int half_samples = samples / 2;
                int first_fourth = samples / 4;
                int third_fourth = half_samples + first_fourth;
                for (int i = 0; i < samples; i++) {
                    dest->freq_tables[t][i] = (i <= first_fourth) ?
                        2048 * ((float)(-i) / (float) samples) :
                        (i <= third_fourth) ? 2048 * ((float)(i-half_samples) / (float) samples) : 2048 * ((float)(samples-i) / (float) samples);
                    dest->freq_tables[t][i] *= 4;
                }
            }
            break;
            default:
            case CLARINET: {
                serialPrint("clarinetgen\n");
                for (int i = 0; i < samples; i++) {
                        float harmonic_sample =  sin(2.0 * PI * (float)i / ((float) samples));
                        harmonic_sample += sin(2.0 * PI * (float)i * 3 / ((float) samples)) / 3;
                        harmonic_sample += sin(2.0 * PI * (float)i * 7 / ((float) samples)) / 7;
                        harmonic_sample += sin(2.0 * PI * (float)i * 8 / ((float) samples)) / 8;
                        dest->freq_tables[t][i] = (2048 * harmonic_sample);
                }
            }
            break;
        }
    }
}

void set_output_waveform(WaveType wave){

    if(output_wavetype == wave){
        return;
    }
    output_wavetype = wave;

    for (int t = 0; t < 12; t++) {
        uint32_t samples =  LUT_freq_sizes[t];
        for (int i = 0; i < samples; i++) {
                output_LUT.freq_tables[t][i] = waveform_LUTs[wave].freq_tables[t][i];
        }
    }

}

void display_wave(u8g2_t* u8g2, uint16_t x, uint16_t y){
    switch (output_wavetype) {
        case SAWTOOTH: {
            u8g2_SetFont(u8g2, u8g2_font_7x13_t_symbols);
            u8g2_DrawUTF8(u8g2, x, y, "//");
            break;
        }
        break;
        case SINE: {
            u8g2_SetFont(u8g2, u8g2_font_7x13_t_symbols);
            u8g2_DrawUTF8(u8g2, x, y, "◠◡");
        }
        break;
        case SQUARE: {
            u8g2_SetFont(u8g2, u8g2_font_7x13_t_symbols);
            u8g2_DrawUTF8(u8g2, x, y, " \u25a0");
        }
        break;
        case TRIANGLE: {
            u8g2_SetFont(u8g2, u8g2_font_7x13_t_symbols);
            u8g2_DrawUTF8(u8g2, x, y, " \u25b2");
        }
        break;
        default:
        case CLARINET: {
            u8g2_SetFont(u8g2, u8g2_font_7x13_t_symbols);
            u8g2_DrawUTF8(u8g2, x, y, " \u265b");
        }
        break;
    }
}

inline void synthesize_output(uint16_t keys, uint8_t volume, uint8_t octave, bool first_half){

    // Wavetables are generated assuming that the step size is 2.
    int8_t step_octave = 1 << (octave - 1);
    int sample_begin, sample_end;
    if (first_half) {
        sample_begin = 0;
        sample_end = OUTPUT_SAMPLES/2;
    } else {
        sample_begin = OUTPUT_SAMPLES/2;
        sample_end = OUTPUT_SAMPLES;
    }

    bool keys_pressed [12];
    for (int k = 0; k < 12; k++) {
        keys_pressed[k] = ~keys & ( 1 << (k));
    }


    for (int i = sample_begin; i < sample_end; i++) {
        int32_t out = 0;

        for (int t = 0; t < 12; t++){

            if (keys_pressed[t]) {
                output_LUT_indices[t] = (output_LUT_indices[t] + (step_octave)) % LUT_freq_sizes[t];
                out += output_LUT.freq_tables[t][output_LUT_indices[t]];
            }
        }
        output_wave[i] = ((uint16_t)(out >> (12 - volume))) + 2048; // VOLUME to be added here
    }


    //HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
}
