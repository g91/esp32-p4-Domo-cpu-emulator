/*
 * M68K SDK - Audio Implementation
 */

#include "m68k_audio.h"
#include "m68k_io.h"

/* Octave 4 base frequencies (C4=262 through B4=494) */
static const uint32_t note_freq_base[12] = {
    262, 277, 294, 311, 330, 349,   /* C, C#, D, D#, E, F */
    370, 392, 415, 440, 466, 494    /* F#, G, G#, A, A#, B */
};

void audio_init(uint32_t rate, uint32_t channels, uint32_t bits) {
    IO_WRITE32(AUD_SAMPLE_RATE, rate);
    IO_WRITE32(AUD_CHANNELS, channels);
    IO_WRITE32(AUD_BITS, bits);
    IO_WRITE32(AUD_COMMAND, AUD_CMD_INIT);
}

void audio_set_volume(uint32_t vol) {
    IO_WRITE32(AUD_VOLUME, vol);
}

void audio_set_volume_lr(uint32_t left, uint32_t right) {
    IO_WRITE32(AUD_VOLUME_L, left);
    IO_WRITE32(AUD_VOLUME_R, right);
}

void audio_beep(uint32_t freq, uint32_t duration_ms) {
    IO_WRITE32(AUD_BUF_SIZE, freq);
    IO_WRITE32(AUD_BUF_POS, duration_ms);
    IO_WRITE32(AUD_COMMAND, AUD_CMD_BEEP);
}

void audio_tone(uint32_t freq, uint32_t duration_ms) {
    IO_WRITE32(AUD_SAMPLE_RATE, freq);
    IO_WRITE32(AUD_BUF_SIZE, duration_ms);
    IO_WRITE32(AUD_COMMAND, AUD_CMD_TONE);
}

void audio_play(void) {
    IO_WRITE32(AUD_COMMAND, AUD_CMD_PLAY);
}

void audio_stop(void) {
    IO_WRITE32(AUD_COMMAND, AUD_CMD_STOP);
}

void audio_pause(void) {
    IO_WRITE32(AUD_COMMAND, AUD_CMD_PAUSE);
}

void audio_resume(void) {
    IO_WRITE32(AUD_COMMAND, AUD_CMD_RESUME);
}

uint32_t audio_status(void) {
    return IO_READ32(AUD_STATUS);
}

bool audio_is_playing(void) {
    return (IO_READ32(AUD_STATUS) & AUD_S_PLAYING) != 0;
}

/* ---- SB16 DSP ---- */

bool sb16_reset(void) {
    IO_WRITE32(AUD_SB_RESET, 1);
    /* Wait for 0xAA response */
    for (int i = 0; i < 1000; i++) {
        if (IO_READ32(AUD_SB_RSTATUS) & 0x80) {
            if (IO_READ32(AUD_SB_READ) == 0xAA)
                return true;
        }
    }
    return false;
}

uint16_t sb16_version(void) {
    sb16_dsp_write(0xE1);  /* Get DSP version */
    uint8_t hi = sb16_dsp_read();
    uint8_t lo = sb16_dsp_read();
    return ((uint16_t)hi << 8) | lo;
}

void sb16_dsp_write(uint8_t val) {
    /* Wait for DSP ready (bit 7 of write status = 0) */
    while (IO_READ32(AUD_SB_WSTATUS) & 0x80) { /* spin */ }
    IO_WRITE32(AUD_SB_WRITE, val);
}

uint8_t sb16_dsp_read(void) {
    /* Wait for data available (bit 7 of read status = 1) */
    while (!(IO_READ32(AUD_SB_RSTATUS) & 0x80)) { /* spin */ }
    return (uint8_t)IO_READ32(AUD_SB_READ);
}

bool sb16_data_available(void) {
    return (IO_READ32(AUD_SB_RSTATUS) & 0x80) != 0;
}

void sb16_mixer_write(uint8_t reg, uint8_t val) {
    IO_WRITE32(AUD_SB_MIXER_ADDR, reg);
    IO_WRITE32(AUD_SB_MIXER_DATA, val);
}

uint8_t sb16_mixer_read(uint8_t reg) {
    IO_WRITE32(AUD_SB_MIXER_ADDR, reg);
    return (uint8_t)IO_READ32(AUD_SB_MIXER_DATA);
}

/* ---- Musical Note Helpers ---- */

uint32_t audio_note_freq(int note, int octave) {
    if (note < 0 || note > 11) return 0;
    if (octave < 1) octave = 1;
    if (octave > 8) octave = 8;

    uint32_t freq = note_freq_base[note];
    int diff = octave - 4;
    if (diff > 0) {
        while (diff-- > 0) freq *= 2;
    } else if (diff < 0) {
        while (diff++ < 0) freq /= 2;
    }
    return freq;
}

void audio_play_note(int note, int octave, uint32_t duration_ms) {
    uint32_t freq = audio_note_freq(note, octave);
    if (freq > 0)
        audio_beep(freq, duration_ms);
}
