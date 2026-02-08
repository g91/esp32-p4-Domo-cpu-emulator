/*
 * M68K SDK - Audio API (Sound Blaster 16)
 * ========================================
 * Sound output via the SB16-compatible audio device on the bus controller.
 * Supports simple beep/tone generation and SB16 DSP programming.
 */

#ifndef M68K_AUDIO_H
#define M68K_AUDIO_H

#include "m68k_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the audio device.
 * rate: sample rate in Hz (e.g. 22050, 44100)
 * channels: 1 (mono) or 2 (stereo)
 * bits: 8 or 16 */
void audio_init(uint32_t rate, uint32_t channels, uint32_t bits);

/* Set master volume (0-255) */
void audio_set_volume(uint32_t vol);

/* Set left/right channel volumes (0-255 each) */
void audio_set_volume_lr(uint32_t left, uint32_t right);

/* Play a beep tone.
 * freq: frequency in Hz, duration_ms: duration in milliseconds */
void audio_beep(uint32_t freq, uint32_t duration_ms);

/* Play a tone (alternate command).
 * freq: frequency, duration_ms: duration */
void audio_tone(uint32_t freq, uint32_t duration_ms);

/* Start playback of the audio DMA buffer */
void audio_play(void);

/* Stop playback */
void audio_stop(void);

/* Pause playback */
void audio_pause(void);

/* Resume playback */
void audio_resume(void);

/* Get audio device status. Returns AUD_S_* bit flags. */
uint32_t audio_status(void);

/* Check if audio is currently playing */
bool audio_is_playing(void);

/* ---- SB16 DSP Low-Level Interface ---- */

/* Reset the SB16 DSP. Returns true if DSP responded with 0xAA. */
bool sb16_reset(void);

/* Get DSP version (returns major<<8 | minor, e.g. 0x0405 = v4.5) */
uint16_t sb16_version(void);

/* Write a command byte to the DSP (waits for ready) */
void sb16_dsp_write(uint8_t val);

/* Read a data byte from the DSP (waits for data available) */
uint8_t sb16_dsp_read(void);

/* Check if DSP has data available to read */
bool sb16_data_available(void);

/* Set a SB16 mixer register */
void sb16_mixer_write(uint8_t reg, uint8_t val);

/* Read a SB16 mixer register */
uint8_t sb16_mixer_read(uint8_t reg);

/* ---- Musical Note Helpers ---- */

/* Note frequency table (octave 4 base, C4-B4) */
#define NOTE_C  0
#define NOTE_CS 1
#define NOTE_D  2
#define NOTE_DS 3
#define NOTE_E  4
#define NOTE_F  5
#define NOTE_FS 6
#define NOTE_G  7
#define NOTE_GS 8
#define NOTE_A  9
#define NOTE_AS 10
#define NOTE_B  11

/* Get frequency for a note at a given octave (1-8).
 * note: NOTE_C through NOTE_B, octave: 1-8.
 * Returns frequency in Hz. */
uint32_t audio_note_freq(int note, int octave);

/* Play a musical note.
 * note: NOTE_C through NOTE_B, octave: 1-8, duration_ms. */
void audio_play_note(int note, int octave, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* M68K_AUDIO_H */
