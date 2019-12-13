#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>


struct {
  uint16_t count;
  uint16_t bits[64];
  uint16_t stop_bit;
} adb_send_data =
  {
   8, { 64, 64, 64, 64, 6400, 6400, 6400, 6400,
	64, 64, 64, 64, 6400, 6400, 6400, 6400,
	64, 64, 64, 64, 6400, 6400, 6400, 6400,
	64, 64, 64, 64, 6400, 6400, 6400, 6400,
	64, 64, 64, 64, 6400, 6400, 6400, 6400,
	64, 64, 64, 64, 6400, 6400, 6400, 6400,
	64, 64, 64, 64, 6400, 6400, 6400, 6400,
	64, 64, 64, 64, 6400, 6400, 6400, 6400 }, 64
  };

typedef void(*irq_t)(void);

void init_adb(void);
void adb_prepare_output(void);
void adb_prepare_input(void);
void adb_send_bytes(irq_t cb);
void attn_sync(void);

void data_done(void);


void data_done (void) {
  for (uint32_t i = 0; i < 8000000; i++)	/* Wait a bit. */
    __asm__("nop");

  while(1) {
    gpio_toggle(GPIOB, GPIO1);	/* LED on/off */

    for (uint32_t i = 0; i < 800000; i++)	/* Wait a bit. */
      __asm__("nop");
  }
}

void tim3_isr(void) {
  if (timer_get_flag(TIM3, TIM_SR_UIF)) {
    timer_clear_flag(TIM3, TIM_SR_UIF);
    
    gpio_clear(GPIOB, GPIO1);
  }
  if (timer_get_flag(TIM3, TIM_SR_CC4IF)) {
    timer_clear_flag(TIM3, TIM_SR_CC4IF);
    gpio_set(GPIOB, GPIO1);
  }
}

void dma1_channel3_isr(void) {
  if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_TCIF)) {
    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL3, DMA_TCIF);
  }
}

int main(void) {

  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  rcc_periph_clock_enable(RCC_TIM3);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_DMA1);

  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO1);

  timer_disable_counter(TIM3);
  dma_disable_channel(DMA1, DMA_CHANNEL3);
  
  // reset the timer
  rcc_periph_reset_pulse(RST_TIM3);

  // Set up the timer baseline clock, direction etc
  timer_set_prescaler(TIM3, ((rcc_apb1_frequency * 2) / 5000)); // 5khz?
  timer_set_clock_division(TIM3, 1);
  timer_continuous_mode(TIM3);
  timer_direction_down(TIM3);
  timer_enable_preload(TIM3);

  // Set up for 1Hz period, direct output compare
  timer_set_period(TIM3, 6464);                                 // 6464 5kHz ticks now.
  timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);
  timer_set_oc_fast_mode(TIM3, TIM_OC4);
  timer_enable_oc_preload(TIM3, TIM_OC4);
  timer_set_oc_polarity_high(TIM3, TIM_OC4);
  timer_enable_oc_output(TIM3, TIM_OC4);
  timer_set_oc_value(TIM3, TIM_OC4, 3232);

  // Set up dma
  dma_channel_reset(DMA1, DMA_CHANNEL3);
  dma_set_priority(DMA1, DMA_CHANNEL3, DMA_CCR_PL_HIGH);
  dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_16BIT);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_16BIT);
  
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);
  dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL3);
  //dma_enable_circular_mode(DMA1, DMA_CHANNEL3);
  
  dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
  
  dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t)&TIM3_CCR4); // direct mode
  //dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t)&TIM3_DMAR); // burst mode
  dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)&(adb_send_data.bits[0]));
  dma_set_number_of_data(DMA1, DMA_CHANNEL3, 65);
  
  //nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);
  //dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);

  // enable dma on the timer side.
  timer_enable_irq(TIM3, TIM_DIER_CC4DE);
  //TIM_DCR(TIM3) = (1 << 8) | 17; // 1 transfer per burst, to CCR4
  
  //  timer_enable_irq(TIM3, TIM_DIER_UIE);
  //  timer_enable_irq(TIM3, TIM_DIER_CC4IE);
  
  dma_enable_channel(DMA1, DMA_CHANNEL3);
  timer_enable_counter(TIM3);
  
  while (1) {
    for (uint32_t i = 0; i < 800000; i++)	/* Wait a bit. */
      __asm__("nop");
  }
  
  return 0;
}

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

void init_adb() {
  // Enable the clocks
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_TIM3);
  rcc_periph_clock_enable(RCC_DMA1);

  rcc_periph_reset_pulse(RST_TIM3);
  
  timer_set_prescaler(TIM3, 71);
  timer_set_period(TIM3, 100);
  timer_set_clock_division(TIM3, 1);
  timer_enable_preload(TIM3);
}

void adb_prepare_output() {
  // Initialise the pin
  gpio_set_mode(GPIO_BANK_TIM3_CH4, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_TIM3_CH4);
}

void adb_prepare_input() {
  // Initialise the pin
  gpio_set_mode(GPIO_BANK_TIM3_CH4, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_TIM3_CH4);
}



// This will send one start bit, /count/ sets of 8 bits, and a stop bit. 
void adb_send_bytes(irq_t cb) {  
  // Set up the timer for PWM, 100 µsec period, initial 1 start pulse
  timer_disable_counter(TIM3);
  dma_disable_channel(DMA1, DMA_CHANNEL3);

  timer_continuous_mode(TIM3);
  timer_direction_down(TIM3);
  timer_set_period(TIM3, 100);
  timer_enable_oc_preload(TIM3, TIM_OC4);
  timer_set_counter(TIM3, 100);
  timer_set_oc_value(TIM3, TIM_OC4, 65);  // We want 35 µsec pulse, but we count down from 100

  dma_set_peripheral_address(DMA1, DMA_CHANNEL3, TIM3_CCR4);
  dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)(adb_send_data.bits));
  dma_set_number_of_data(DMA1, DMA_CHANNEL3, (adb_send_data.count << 3) + 1);
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);
  //  dma1_channel3_isr = cb;
  
  timer_set_dma_on_update_event(TIM3);
  
  dma_enable_channel(DMA1, DMA_CHANNEL3);
  timer_enable_counter(TIM3);
}

void attn_sync() {
}

