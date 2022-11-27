#include "adb.h"
#include "util.h"
#include "usb.h"

void adb_bit_send_receive_done (void);

#define ADB_TALK(d,r)   (((d) & 0xf) << 4) | 0x0c | ((r) & 0x3)
#define ADB_LISTEN(d,r) (((d) & 0xf) << 4) | 0x08 | ((r) & 0x3)
#define MAX_RX 66 * 2

struct {
  uint16_t count;
  struct {
    uint16_t low;
    uint16_t high;
  } receive_data [66];
  uint16_t send_data [65];
} bit_timings;


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
// bytes of data, and a single "0" stop bit as above.  The device's desire to
// send us data is signalled by pulling the line low during SRQ.

// The "fixed" parts can all be generated using one pulse mode
// The bytes of data, on the other hand, we will generate using DMA and
// output compare mode.  Or maybe pwm mode.  Not sure about that.

// We will be using timer 2 channel 1 for all this, remapped onto PA15, which
// is 5v tolerant.

mutex_t _adb_mutex = MUTEX_UNLOCKED;
uint16_t prescale;

void adb_common_setup (void) {
  dma_channel_reset   (DMA1, DMA_CHANNEL5);
  gpio_primary_remap  (AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, AFIO_MAPR_TIM2_REMAP_NO_REMAP); 
  gpio_primary_remap  (AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, AFIO_MAPR_TIM2_REMAP_PARTIAL_REMAP1); 
  nvic_enable_irq     (NVIC_DMA1_CHANNEL5_IRQ);
  nvic_enable_irq     (NVIC_TIM2_IRQ);

  // Set up pin as input, pulled high, so the line is correct.
  gpio_set_mode       (GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO15);
  GPIOA_ODR |= GPIO15;

  // Set up for a 1usec clock tick 
  TIM2_PSC = 71;
}

void adb_attn (void) {
  gpio_set_mode  (GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
  gpio_clear (GPIOA, GPIO15);
  usleep(800);
}

void adb_start (void) {
  gpio_set_mode  (GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
  gpio_clear (GPIOA, GPIO15);
  usleep(_adb_1_bit_low_time);
  gpio_set (GPIOA, GPIO15);
  usleep(_adb_1_bit_high_time); 
}

void adb_sync (void) {
  gpio_set_mode  (GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
  gpio_clear (GPIOA, GPIO15);
  usleep(_adb_0_bit_low_time);
  gpio_set (GPIOA, GPIO15);
  usleep(_adb_0_bit_high_time); 
}

/* // Send a byte over the ADB bus */
/* void adb_send_byte (uint8_t data) { */
/*   for (uint8_t i = 0; i < 8; i++) { */
/*     uint8_t b = (data << i) & 0x80;  */
/*     gpio_clear (GPIOA, GPIO15); */
/*     usleep(b ? _adb_1_bit_low_time : _adb_0_bit_low_time); */
/*     gpio_set (GPIOA, GPIO15); */
/*     usleep(b ? _adb_1_bit_high_time : _adb_0_bit_high_time); */
/*   } */
/* } */

void adb_listen (uint8_t dev, uint8_t reg, uint8_t *count, uint8_t * data) {

  // Sequence of commands for an ADB listen command
  adb_attn      ();
  adb_start     ();
  adb_send_byte (ADB_LISTEN(dev, reg));
  adb_sync      ();
  usleep        (200);
  adb_start     ();
  for (uint8_t i = 0; i < *count; i++) {
    adb_send_byte (data[i]);
  }
  adb_sync();
  gpio_set_mode (GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO15);
}

void adb_talk (uint8_t dev, uint8_t reg, uint8_t *count, uint8_t * data) {
  // Sequence of commands for an ADB listen command
  adb_attn         ();
  adb_start        ();
  adb_send_byte    (ADB_TALK(dev, reg));
  adb_sync         ();
  gpio_set_mode    (GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO15);
  adb_receive_data (count, data);
}



void dma1_channel5_isr (void) {
  if (dma_get_interrupt_flag (DMA1, DMA_CHANNEL5, DMA_TCIF)) {
    dma_clear_interrupt_flags (DMA1, DMA_CHANNEL5, DMA_TCIF);
  }
  
  dma_disable_channel (DMA1, DMA_CHANNEL5);
  timer_clear_flag (TIM2, TIM_SR_UIF);
  TIM2_DIER = TIM_DIER_UIE;    
}

void tim2_isr (void) {
  if (timer_get_flag (TIM2, TIM_SR_CC3IF)) {
    timer_clear_flag (TIM2, TIM_SR_CC3IF);
  }
  if (timer_get_flag (TIM2, TIM_SR_CC1IF)) {
    timer_clear_flag (TIM2, TIM_SR_CC1IF);
  }
  if (timer_get_flag (TIM2, TIM_SR_UIF)) {
    timer_clear_flag (TIM2, TIM_SR_UIF);
  }
    timer_disable_counter (TIM2);

    adb_bit_send_receive_done ();

}

void adb_bit_send_receive_done (void) {
  gpio_set            (GPIOA, GPIO15);
  mutex_unlock        (&_adb_mutex);
}


void adb_send_byte (uint8_t data) {
  uint16_t timings[8] = {0, 0, 0, 0, 0, 0, 0, 0 };

  for (int i = 0; i < 8; i++) {
    timings[i] = ((data << i) & 0x80) ? _adb_1_bit_high_time : _adb_0_bit_high_time;
  }
  
  /* usb_vcp_printf("d : %u %u %u %u %u %u %u %u\r", timings[0], timings[1], timings[2], timings[3], timings[4], timings[5], timings[6], timings[7]); */
  /* usleep(2000000); */
  /* return; */
  
  mutex_lock(&_adb_mutex);
  
  gpio_set_mode  (GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO15);
  
  // Timer counts up, internal clock, preload enabled 
  TIM2_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_ARPE | TIM_CR1_DIR_DOWN;
  TIM2_CR2 = 0;// TIM_CR2_CCDS;
  TIM2_SMCR = 0;
  // DMA transfer on CC1
  TIM2_DIER = TIM_DIER_CC1DE;
  TIM2_EGR = TIM_EGR_CC1G;

  // PWM Mode 2 i.e. low to high on capture
  // Preload enabled, fast compare enabled, output.
  TIM2_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE | TIM_CCMR1_CC1S_OUT | TIM_CCMR1_OC1M_PWM2;
  TIM2_CCMR2 = 0;

  // Enable channel 1
  TIM2_CCER = TIM_CCER_CC1E | TIM_CCER_CC1P;

  // Set up count and so on
  TIM2_CNT = 1;
  TIM2_ARR = _adb_bit_length;
  TIM2_CCR1 = timings[0];

  // DMA Stuff
  TIM2_DCR = ((1 - 1) << 4) | (0x34 >> 2);
  TIM2_DMAR = 0;

  // And DMA 1 Channel 5
  // 16 bit transfers, memory to peripheral, high priority, increment memory each tx, interrupt on completion
  DMA1_CCR5 = DMA_CCR_MINC | DMA_CCR_MSIZE_16BIT | DMA_CCR_PSIZE_16BIT | DMA_CCR_PL_HIGH | DMA_CCR_TCIE | DMA_CCR_DIR;
  DMA1_CNDTR5 = 9;
  DMA1_CMAR5 = (uint32_t)&(timings[0]);
  DMA1_CPAR5 = (uint32_t)&TIM2_DMAR;


  dma_enable_channel (DMA1, DMA_CHANNEL5);
  timer_enable_counter (TIM2);
  
  mutex_lock          (&_adb_mutex);
  mutex_unlock          (&_adb_mutex);

 
}


// This does everything via direct register access, which speeds things up considerably
// To receive data, we use, once again, Timer 2.  Channel 1 and 2 are set up to do input capture
// with channel 1 capturing the low->high transition and channel 2 capturing high->low.
// Channel 3 is set up for output capture, this handles timeout via an isr.
// We set up slave mode to reset the timer, triggered by channel 2
void adb_receive_bits (void) {
  static const uint8_t burst_length = 2;

  // Lock the mutex, we're working
  mutex_lock    (&_adb_mutex);

  // Set our GPIO to input, pull up.
  gpio_set_mode (GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO15);
  GPIOA_ODR |= GPIO15;

  // Timer counts up, internal clock, preload enabled
  TIM2_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_ARPE | TIM_CR1_DIR_UP;
  TIM2_CR2 = 0;

  // Slave mode trigger on channel 2, trigger generates reset 
  TIM2_SMCR = TIM_SMCR_TS_TI2FP2 | TIM_SMCR_SMS_RM;

  // enable trigger DMA transfer and interrupt on capture-compare channel 3
  TIM2_DIER = TIM_DIER_TDE | TIM_DIER_CC3IE;
  // Disable all capture-compares so we can mess with the settings
  TIM2_CCER = 0;
  // CC1 & CC2 set up to capture PWM on TI1 with a x4 filter (4 µsec delay)
  TIM2_CCMR1 = TIM_CCMR1_CC2S_IN_TI1 | TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_CK_INT_N_4 | TIM_CCMR1_IC2F_CK_INT_N_4;
  // CC3 will count up to the value in CCR3, preload and fast enabled, no output to its pin
  TIM2_CCMR2 = TIM_CCMR2_OC3M_FROZEN | TIM_CCMR2_OC3PE | TIM_CCMR2_OC3FE | TIM_CCMR2_CC3S_OUT;
  // enable the 3 capture ompare channels, invert polarity on CC2
  TIM2_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC2P;
  // Reset the counter
  TIM2_CNT = 0;
  // Set the period to something we'll never reach
  TIM2_ARR = 6000;
  // CC3 timeout set to 200 µsec, i.e 2 bit frames.  This is the same as the SRQ period, but we should start about half way through
  TIM2_CCR3 = 200;
  // Set up dma for bursts of 2, CC1 & CC2, which is where our bit timing data should reside
  TIM2_DCR = ((burst_length & 0x1f) << 8) | (0x34 >> 2);
  TIM2_DMAR = 0;

  // Set up DMA to do 16 bit transfers, incrementing the memory address every time, interrupt on transfer complete, priority high
  DMA1_CCR5 = DMA_CCR_PL_HIGH | DMA_CCR_MSIZE_16BIT | DMA_CCR_PSIZE_16BIT | DMA_CCR_MINC | DMA_CCR_TCIE;
  // Maximum transfers 8 * 8 bits + start & stop, 2 values per bit
  DMA1_CNDTR5 = MAX_RX;
  // Use the DMAR register as source, thus doing burst mode DMA
  DMA1_CPAR5 = (uint32_t)&TIM2_DMAR;
  DMA1_CMAR5 = (uint32_t)&(bit_timings.receive_data[0]);

  // Kick it all off!
  dma_enable_channel (DMA1, DMA_CHANNEL5);
  timer_enable_counter (TIM2);
}
  

  
