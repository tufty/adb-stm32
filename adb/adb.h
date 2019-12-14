#ifndef __ADB_H__
#define __ADB_H__ 1

// Defines for the various constants we need.
static const enum rcc_periph_clken _rcc_adb_timer = RCC_TIM2;
static const enum rcc_periph_clken _rcc_adb_gpio = RCC_GPIOA;
static const enum rcc_periph_clken _rcc_adb_dma = RCC_DMA1;

static const uint32_t _adb_gpio = GPIOA;
static const uint32_t _adb_pin = GPIO15;

static const uint32_t _adb_timer = TIM2;
static const enum tim_oc_id _adb_oc_id = TIM_OC1;
static const uint32_t _adb_ccr = TIM2 + 34;
static const uint32_t _adb_timer_dma_enable = TIM_DIER_CC1DE;
static const uint32_t _adb_timer_prescaler = 71;
static const uint32_t _adb_timer_clock_division = 1;

static const uint32_t _adb_dma = DMA1;
static const uint8_t _adb_dma_channel = DMA_CHANNEL5;
static const uint8_t _adb_dma_irq = NVIC_DMA1_CHANNEL5_IRQ;

static const uint32_t _adb_bit_length = 100;
static const uint16_t _adb_1_bit_time = 65;
static const uint16_t _adb_0_bit_time = 35;


// Super low level functions
void adb_send_bits (uint8_t count, uint16_t * bit_timings);
void adb_receive_bits (uint8_t * count, uint16_t * bit_timings);

// API functions
void adb_attn (void);

#endif /* __ADB_H__ */
