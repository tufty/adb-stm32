#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/sync.h>

#include "adb.h"

void adb_bit_send_done (void);

#define ADB_TALK (device, register)   (((device) & 0xf) << 4) | 0x0c | ((register) & 0x3)
#define ADB_LISTEN (device, register) (((device) & 0xf) << 4) | 0x08 | ((register) & 0x3)

// Timer setup for dma-based output compare to generate ADB signalling.
// Every command has the same prelude, viz:
// Signal high to start with.
// pull signal low for 800 µsec (ATTN)
// drive signal high for 70 µsec (SYNC)

// We then have a train of 8 bits, each 100 µsec long.  Duty cycle
// indicates content.

// There is then a 65 µsec low "stop", and we go to input mode for
// 200 µsec.  This is the SRQ period, if we are asking for data the line
// will be pulled low by the device

// We may then send data to the device.  This involves a single "1" start bit
// 35 µsec low, 65 µsec high, 2 to 8 bytes of data, and a single "0" stop bit
// 65 µsec low, 35 µsec high.

// If the device sends data to us, we expect a single "1" start bit, 2 to 8
// bytes of data, and a single "0" stop bit as above.

// The "fixed" parts can all be generated using one pulse mode
// The bytes of data, on the other hand, we will generate using DMA and
// output compare mode.  Or maybe pwm mode.  Not sure about that.

// We will be using timer 2 channel 1 for all this, remapped onto PA15, which
// is 5v tolerant.

mutex_t _adb_mutex = MUTEX_UNLOCKED;

void dma1_channel5_isr (void) {
  if (dma_get_interrupt_flag (_adb_dma, _adb_dma_channel, DMA_TCIF)) {
    dma_clear_interrupt_flags (_adb_dma, _adb_dma_channel, DMA_TCIF);
  }
  adb_bit_send_done ();
}

void adb_bit_send_done (void) {
  timer_disable_counter (_adb_timer);
  dma_disable_channel   (_adb_dma, _adb_dma_channel);
  mutex_unlock          (&_adb_mutex);
}

// Send bits. 
void adb_send_bits (uint8_t count, uint16_t * bit_timings) {
  mutex_lock                (&_adb_mutex);

  timer_disable_counter     (_adb_timer);
  dma_disable_channel       (_adb_dma, _adb_dma_channel);

  gpio_set_mode(_adb_gpio, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, _adb_pin);

  timer_set_prescaler       (_adb_timer, _adb_timer_prescaler);
  timer_set_clock_division  (_adb_timer, _adb_timer_clock_division);
  timer_continuous_mode     (_adb_timer);
  timer_direction_down      (_adb_timer);
  timer_enable_preload      (_adb_timer);
  timer_set_period          (_adb_timer, _adb_bit_length);
  
  timer_set_oc_mode         (_adb_timer, _adb_oc_id, TIM_OCM_PWM1);
  timer_set_oc_fast_mode    (_adb_timer, _adb_oc_id);
  timer_enable_oc_preload   (_adb_timer, _adb_oc_id);
  timer_set_oc_polarity_low (_adb_timer, _adb_oc_id);
  timer_enable_oc_output    (_adb_timer, _adb_oc_id);
  
  timer_set_oc_value        (_adb_timer, _adb_oc_id, _adb_1_bit_time);

  dma_channel_reset         (_adb_dma, _adb_dma_channel);
  dma_set_priority          (_adb_dma, _adb_dma_channel, DMA_CCR_PL_HIGH);
  dma_set_memory_size       (_adb_dma, _adb_dma_channel, DMA_CCR_MSIZE_16BIT);
  dma_set_peripheral_size   (_adb_dma, _adb_dma_channel, DMA_CCR_PSIZE_16BIT);

  dma_enable_memory_increment_mode      (_adb_dma, _adb_dma_channel);
  dma_disable_peripheral_increment_mode (_adb_dma, _adb_dma_channel);
  dma_set_read_from_memory              (_adb_dma, _adb_dma_channel);
  dma_set_peripheral_address            (_adb_dma, _adb_dma_channel, (uint32_t)_adb_ccr);
  dma_set_memory_address                (_adb_dma, _adb_dma_channel, (uint32_t)bit_timings);
  dma_set_number_of_data                (_adb_dma, _adb_dma_channel, count);

  nvic_enable_irq                        (_adb_dma_irq);
  dma_enable_transfer_complete_interrupt (_adb_dma, _adb_dma_channel);

  timer_enable_irq                       (_adb_timer, _adb_timer_dma_enable);
  
  dma_enable_channel                     (_adb_dma, _adb_dma_channel);
  timer_enable_counter                   (_adb_timer);

}
