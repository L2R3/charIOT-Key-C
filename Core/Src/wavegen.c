#include <wavegen.h>

volatile WaveType output_wavetype = END_WAVETYPE;

// Lookup tables for the different types of wave that can be produced
// e.g. sine, sawtooth, clarinet etc.
int16_t DDS_LUT[END_WAVETYPE][DDS_LUT_SAMPLES];
int16_t DDS_LUT_SEL[DDS_LUT_SAMPLES];

uint32_t DDS_indices[12] = { 0 };
uint32_t DDS_steps[12];

uint16_t DDS_OUT[DDS_OUT_SAMPLES];

// Initialise the contents of the all the LUTs
// This means generating the different instrument waveforms
void init_lookup_tables() {

	for (int t = 0; t < 12; t++) {
		DDS_steps[t] = 3520.0 * pow(2, (t - 9) / 12.0) / 44100 * 65536.0;
	}

	for (WaveType type = 0; type < END_WAVETYPE; type++) {
		generate_waveform(DDS_LUT[type], type);
	}
}

void generate_waveform(int16_t lookup_table[DDS_LUT_SAMPLES], WaveType wave) {

	int half_samples = DDS_LUT_SAMPLES / 2;

	switch (wave) {
	case SAWTOOTH: {
		for (int i = 0; i < DDS_LUT_SAMPLES; i++) {
			lookup_table[i] =
					(i <= half_samples) ?
							2048
									* ((float) (i - half_samples)
											/ (float) DDS_LUT_SAMPLES) :
							2048
									* ((float) (i - half_samples)
											/ (float) DDS_LUT_SAMPLES);
		}
	}
		break;
	case SINE: {
		for (int i = 0; i < DDS_LUT_SAMPLES; i++) {
			lookup_table[i] = 2048
					* sin(2.0 * M_PI * (float) i / (float) DDS_LUT_SAMPLES);
		}
	}
		break;
	case SQUARE: {
		for (int i = 0; i < DDS_LUT_SAMPLES; i++) {
			lookup_table[i] =
					(i <= half_samples) ? 2048 * (1.0) : 2048 * (-1.0);
		}
	}
		break;
	case TRIANGLE: {
		int half_samples = DDS_LUT_SAMPLES / 2;
		int first_fourth = DDS_LUT_SAMPLES / 4;
		int third_fourth = half_samples + first_fourth;
		for (int i = 0; i < DDS_LUT_SAMPLES; i++) {
			lookup_table[i] =
					(i <= first_fourth) ?
							2048 * ((float) (-i) / (float) DDS_LUT_SAMPLES) :
					(i <= third_fourth) ?
							2048
									* ((float) (i - half_samples)
											/ (float) DDS_LUT_SAMPLES) :
							2048
									* ((float) (DDS_LUT_SAMPLES - i)
											/ (float) DDS_LUT_SAMPLES);
			lookup_table[i] *= 4;
		}
	}
		break;
	case RETRO1: {
		int tone1 = DDS_LUT_SAMPLES * 0.2;
		int tone2 = DDS_LUT_SAMPLES * 0.4;
		int tone3 = DDS_LUT_SAMPLES * 0.3;
		uint32_t curr_samples = 0;
		uint32_t step_samples = curr_samples + tone1;
		uint32_t half_step_samples = step_samples / 2;

		for (int i = curr_samples; i < step_samples; i++) {
			float wave = 2048
					* sin(2.0 * M_PI * (float) i * 5 / ((float) tone1)) / 5;
			lookup_table[i] = 0.4 * wave;
		}
		curr_samples = step_samples;

		step_samples = curr_samples + tone2;
		half_step_samples = curr_samples + tone2 / 2;
		for (int i = curr_samples; i < step_samples; i++) {
			float wave = 2048 * sin(2.0 * M_PI * (float) i / ((float) tone2));
			lookup_table[i] = 1 * wave;
		}
		curr_samples = step_samples;

		step_samples = curr_samples + tone3;
		half_step_samples = curr_samples + tone3 / 2;
		for (int i = curr_samples; i < step_samples; i++) {
			float wave = 2048 * sin(2.0 * M_PI * (float) i / ((float) tone3));
			lookup_table[i] = 0.7 * wave;
		}
		curr_samples = step_samples;

		half_step_samples = step_samples + (DDS_LUT_SAMPLES - step_samples) / 2;
		for (int i = curr_samples; i < DDS_LUT_SAMPLES; i++) {
			float wave =
					(i <= half_step_samples) ? 2048 * (1.0) : 2048 * (-1.0);
			;
			lookup_table[i] = 0.3 * wave;
		}
	}
		break;
	case RETRO2: {
		int tone1 = DDS_LUT_SAMPLES * 0.4;
		int tone2 = DDS_LUT_SAMPLES * 0.3;
		uint32_t curr_samples = 0;
		uint32_t step_samples = curr_samples + tone1;
		uint32_t half_step_samples = curr_samples + tone1 / 2;
		for (int i = curr_samples; i < step_samples; i++) {
			float wave =
					(i <= half_step_samples) ? 2048 * (1.0) : 2048 * (-1.0);
			wave += 2048 * sin(2.0 * M_PI * (float) i / ((float) tone1));
			lookup_table[i] = 1 * wave;
		}
		curr_samples = step_samples;

		step_samples = curr_samples + tone2;
		half_step_samples = curr_samples + tone2 / 2;
		for (int i = curr_samples; i < step_samples; i++) {
			float wave =
					(i <= half_step_samples) ? 2048 * (1.0) : 2048 * (-1.0);
			lookup_table[i] = 0.7 * wave;
		}
		curr_samples = step_samples;

		half_step_samples = step_samples + (DDS_LUT_SAMPLES - step_samples) / 2;
		for (int i = curr_samples; i < DDS_LUT_SAMPLES; i++) {
			float wave =
					(i <= half_step_samples) ? 2048 * (1.0) : 2048 * (-1.0);
			lookup_table[i] = 0.3 * wave;
		}
	}
		break;
	default:
	case CLARINET: {
		for (int i = 0; i < DDS_LUT_SAMPLES; i++) {
			float harmonic_sample = sin(
					2.0 * M_PI * (float) i / ((float) DDS_LUT_SAMPLES));
			harmonic_sample += sin(
					2.0 * M_PI * (float) i * 3 / ((float) DDS_LUT_SAMPLES)) / 3;
			harmonic_sample += sin(
					2.0 * M_PI * (float) i * 7 / ((float) DDS_LUT_SAMPLES)) / 7;
			harmonic_sample += sin(
					2.0 * M_PI * (float) i * 8 / ((float) DDS_LUT_SAMPLES)) / 8;
			lookup_table[i] = (2048 * harmonic_sample);
		}
	}
		break;
	}
}

void set_output_waveform(WaveType wave) {

	// change only if different
	if (output_wavetype == wave) {
		return;
	}
	output_wavetype = wave;

	// start M2M DMA transfer
	HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel1, (uint32_t) DDS_LUT[wave],
			(uint32_t) DDS_LUT_SEL, DDS_LUT_SAMPLES);

}

void display_wave(u8g2_t *u8g2, uint16_t x, uint16_t y) {
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
	case RETRO1: {
		u8g2_SetFont(u8g2, u8g2_font_7x13_t_symbols);
		u8g2_DrawUTF8(u8g2, x, y, " \u2600");
	}
		break;
	case RETRO2: {
		u8g2_SetFont(u8g2, u8g2_font_7x13_t_symbols);
		u8g2_DrawUTF8(u8g2, x, y, " \u2604");
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

inline void synthesise_output1() {

	// load the keys, protected by Mutex
	uint16_t localKeys[keyboard_count];
	osMutexAcquire(notesMutexHandle, osWaitForever);
	for (int board = 0; board < keyboard_count; board++) {
		localKeys[board] = allKeys[board];
	}
	osMutexRelease(notesMutexHandle);

	// re-shuffle into notes
	uint8_t localNotes[12];
	for (int key = 0; key < 12; key++) {
		localNotes[key] = 0;
		for (int board = 0; board < keyboard_count; board++) {
			localNotes[key] |= ((~(localKeys[board]) >> key) & 1) << board;
		}
	}

	// load the position-octave difference atomically
	uint8_t localDiff = __atomic_load_n(&pos_oct_diff, __ATOMIC_RELAXED);

	// load the volume atomically
	uint8_t localVolume = __atomic_load_n(&volume, __ATOMIC_RELAXED);

	for (int i = 0; i < DDS_OUT_SAMPLES / 2; i++) {
		int32_t out = 0;
		for (int key = 0; key < 12; key++) {
			if (localNotes[key]) {
				DDS_indices[key] += DDS_steps[key];
				for (int board = 0; board < keyboard_count; board++) {
					if (localNotes[key] & (1 << board)) {
						out += DDS_LUT_SEL[(DDS_indices[key]
								>> (13 - board + localDiff)) & 0x03FF];
					}
				}
			}
		}
		DDS_OUT[i] = ((uint16_t) (out >> (12 - localVolume))) + 2048;
	}

}

inline void synthesise_output2() {

	// load the keys, protected by Mutex
	uint16_t localKeys[keyboard_count];
	osMutexAcquire(notesMutexHandle, osWaitForever);
	for (int board = 0; board < keyboard_count; board++) {
		localKeys[board] = allKeys[board];
	}
	osMutexRelease(notesMutexHandle);

	// re-shuffle into notes
	uint8_t localNotes[12];
	for (int key = 0; key < 12; key++) {
		localNotes[key] = 0;
		for (int board = 0; board < keyboard_count; board++) {
			localNotes[key] |= ((~(localKeys[board]) >> key) & 1) << board;
		}
	}

	// load the position-octave difference atomically
	uint8_t localDiff = __atomic_load_n(&pos_oct_diff, __ATOMIC_RELAXED);

	// load the volume atomically
	uint8_t localVolume = __atomic_load_n(&volume, __ATOMIC_RELAXED);

	for (int i = DDS_OUT_SAMPLES / 2; i < DDS_OUT_SAMPLES; i++) {
		int32_t out = 0;
		for (int key = 0; key < 12; key++) {
			if (localNotes[key]) {
				DDS_indices[key] += DDS_steps[key];
				for (int board = 0; board < keyboard_count; board++) {
					if (localNotes[key] & (1 << board)) {
						out += DDS_LUT_SEL[(DDS_indices[key]
								>> (13 - board + localDiff)) & 0x03FF];
					}
				}
			}
		}
		DDS_OUT[i] = ((uint16_t) (out >> (12 - localVolume))) + 2048;
	}

}
