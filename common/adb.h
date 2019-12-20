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

static const uint32_t _adb_bit_length = 100;
static const uint16_t _adb_1_bit_time = 35;
static const uint16_t _adb_0_bit_time = 65;


// Super low level functions
void adb_common_setup (void);
void adb_send_bits (void);
void adb_receive_bits (void);

// API functions
void adb_attn (void);

#endif /* __ADB_H__ */
