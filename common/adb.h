#ifndef __ADB_H__
#define __ADB_H__ 1

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/sync.h>

// Defines for the various constants we need.
static const enum rcc_periph_clken _rcc_adb_timer = RCC_TIM2;
static const enum rcc_periph_clken _rcc_adb_gpio = RCC_GPIOA;
static const enum rcc_periph_clken _rcc_adb_dma = RCC_DMA1;

static const uint16_t _adb_bit_length = 100;
static const uint16_t _adb_byte_length = 1000;
static const uint16_t _adb_1_bit_low_time = 34;
static const uint16_t _adb_0_bit_low_time = 66;
static const uint16_t _adb_1_bit_high_time = 66;
static const uint16_t _adb_0_bit_high_time = 34;

static const uint8_t ADB_TALK = 2;
static const uint8_t ADB_LISTEN = 3;


// Super low level functions
void adb_common_setup (void);
void adb_send_bits (uint16_t count, uint16_t * timing_data);
void adb_receive_bits (void);

// API functions
void adb_attn (void);
void adb_start (void);
void adb_sync (void);
void adb_send_byte (uint8_t data);


void adb_send_command(uint8_t dev, uint8_t cmd, uint8_t reg);

#endif /* __ADB_H__ */
