#include "adb.h"
#include "util.h"
#include "usb.h"

void adb_bit_send_receive_done (void);

#define ADB_TALK(d,r)   (((d) & 0xf) << 4) | 0x0c | ((r) & 0x3)
#define ADB_LISTEN(d,r) (((d) & 0xf) << 4) | 0x08 | ((r) & 0x3)
#define MAX_DATA_BITS (1 + (4 * 8) + 1) * 2

uint16_t timings[MAX_DATA_BITS];
uint8_t timings_cnt;

// State machine state for ADB
enum {
  ADB_SM_IDLE,
  ADB_SM_ATTN,
  ADB_SM_START,
  ADB_SM_SYNC,
  ADB_SM_EMIT,
  ADB_SM_RECV,
  ADB_SM_MAYBE_STOP_TO_START,
  ADB_SM_SRQ,
  ADB_SM_TIMEOUT,
} _adb_sm_state;

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

  _adb_sm_state = ADB_SM_IDLE;
}

void adb_attn (void) {
  _adb_sm_state = ADB_SM_ATTN;
  gpio_set_mode  (GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
  gpio_clear (GPIOA, GPIO15);
  usleep(800);
  _adb_sm_state = ADB_SM_IDLE;
}

void adb_start (void) {
  _adb_sm_state = ADB_SM_START;
  gpio_set_mode  (GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
  gpio_clear (GPIOA, GPIO15);
  usleep(_adb_1_bit_low_time);
  gpio_set (GPIOA, GPIO15);
  usleep(_adb_1_bit_high_time);
  _adb_sm_state = ADB_SM_IDLE;
}

void adb_sync (void) {
  _adb_sm_state = ADB_SM_SYNC;
  gpio_set_mode  (GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
  gpio_clear (GPIOA, GPIO15);
  usleep(_adb_0_bit_low_time);
  gpio_set (GPIOA, GPIO15);
  usleep(_adb_0_bit_high_time);
  _adb_sm_state = ADB_SM_IDLE;
}

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
}

void adb_talk (uint8_t dev, uint8_t reg, uint8_t *count, uint8_t * data) {
  // Sequence of commands for an ADB listen command
  adb_attn         ();
  adb_start        ();
  //  adb_sync         ();
  adb_send_byte    (ADB_TALK(dev, reg));
  adb_sync         ();
  adb_receive_bits(count, data);
}



void dma1_channel5_isr (void) {
  switch (_adb_sm_state) {
  case ADB_SM_EMIT:
    dma_disable_channel (DMA1, DMA_CHANNEL5);
    timer_clear_flag (TIM2, TIM_SR_UIF);
    TIM2_DIER = TIM_DIER_UIE;    
    break;
  case ADB_SM_RECV:
    _adb_sm_state = ADB_SM_IDLE;
    adb_bit_send_receive_done();
    break;
  default:
    break;
  }

  DMA1_IFCR = 0x000f0000;
}

void tim2_isr (void) {
  switch (_adb_sm_state) {
  case ADB_SM_EMIT:
    if (timer_get_flag (TIM2, TIM_SR_UIF)) {
      _adb_sm_state = ADB_SM_IDLE;
      adb_bit_send_receive_done();
    }
    break;
  case ADB_SM_MAYBE_STOP_TO_START:
    if (timer_get_flag (TIM2, TIM_SR_UIF)) {
      _adb_sm_state = ADB_SM_TIMEOUT;
      adb_bit_send_receive_done();
    }
    if (timer_get_flag (TIM2, TIM_SR_CC1IF)) {
      _adb_sm_state = ADB_SM_RECV;
      adb_bit_send_receive_done();
    }
    break;
  case ADB_SM_RECV:
    if (timer_get_flag (TIM2, TIM_SR_UIF)) {
      _adb_sm_state = ADB_SM_TIMEOUT;
      adb_bit_send_receive_done();
    }
    if (timer_get_flag (TIM2, TIM_SR_CC2IF)) {
      timings[timings_cnt++] = TIM2_CCR1;
      TIM2_CNT = 0;
    }
   break;
  default:
    break;
  }

  TIM2_SR = 0;
}

void adb_bit_send_receive_done (void) {
  timer_disable_counter (TIM2);
  dma_disable_channel(DMA1, DMA_CHANNEL5);
  mutex_unlock        (&_adb_mutex);
}


void adb_send_byte (uint8_t data) {

  for (int i = 0; i < 8; i++) {
    timings[i] = ((data << i) & 0x80) ? _adb_1_bit_high_time : _adb_0_bit_high_time;
  }
  
  /* usb_vcp_printf("d : %u %u %u %u %u %u %u %u\r", timings[0], timings[1], timings[2], timings[3], timings[4], timings[5], timings[6], timings[7]); */
  /* usleep(2000000); */
  /* return; */
  _adb_sm_state = ADB_SM_EMIT;
  
  mutex_lock(&_adb_mutex);
  
  gpio_set_mode  (GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO15);
  
  // Timer counts up, internal clock, preload enabled 
  TIM2_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_ARPE | TIM_CR1_DIR_DOWN;
  TIM2_CR2 = TIM_CR2_CCDS;
  TIM2_SR = 0;
  TIM2_SMCR = 0;
  // DMA transfer on CC1
  TIM2_DIER = TIM_DIER_CC1DE;
  TIM2_EGR = TIM_EGR_CC1G;

  // PWM Mode 2 i.e. low to high on capture
  // Preload enabled, fast compare enabled, output.
  TIM2_CCER = 0;
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
  DMA1_CMAR5 = (uint32_t)timings;
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
void adb_receive_bits (uint8_t * count, uint8_t * data) {
 
  // Set our GPIO to input, pull up.
  gpio_set_mode (GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO15);
  GPIOA_ODR |= GPIO15;

  // Lock the mutex, we're working
  mutex_lock    (&_adb_mutex);

  // start by checking if the line is pulled high.  If it isn't, we're in an SRQ
  // situation and the device we're trying to talk to isn't listening or has no
  // data to send.
  if (gpio_get(GPIOA, GPIO15)) {
    _adb_sm_state = ADB_SM_MAYBE_STOP_TO_START;

    // Kick off timer 2 waiting for a high to low transition
    // We don't care about the absolute length of this, but if it
    // runs more than 200 usec we have nobody talking
    TIM2_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_DIR_UP;
    TIM2_CR2 = 0;
    TIM2_SMCR = 0;
    TIM2_DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
    TIM2_CCER = 0;
    TIM2_CCMR1 = TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_CK_INT_N_4;
    TIM2_CCMR2 = 0;
    TIM2_CCER = TIM_CCER_CC1E | TIM_CCER_CC1P;
    TIM2_CNT = 0;
    TIM2_ARR = 200;
    TIM2_SR = 0;

    // Kick it off and wait for results
    timer_enable_counter (TIM2);
    mutex_lock    (&_adb_mutex);

    // Did we transition to low?
    if (ADB_SM_RECV == _adb_sm_state) {
      // Yes, carry on listening to the data
      // At this point we should be at the start of a start bit.
      // Use cc1 & cc2 to do pwm input, LO-HI on CC1 and HI-LO on CC2
      // CC2 also triggers, and trigger sets off DMA
      TIM2_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_DIR_UP | TIM_CR1_URS;
      TIM2_CR2 = 0;
      TIM2_SMCR = 0;
      TIM2_DIER = TIM_DIER_UIE | TIM_DIER_CC2IE;
      TIM2_CCER = 0;
      TIM2_CCMR1 = TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_CK_INT_N_4 | TIM_CCMR1_CC2S_IN_TI1 | TIM_CCMR1_IC2F_CK_INT_N_4;
      TIM2_CCMR2 = 0;
      TIM2_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;
      TIM2_CNT = 0;
      TIM2_ARR = 200;
      TIM2_SR = 0;

      timings_cnt = 0;
      timer_enable_counter (TIM2);

      // Wait
      mutex_lock          (&_adb_mutex);

      // Now unpack the bits.
      // Number of bytes received
      *count = timings_cnt ? (timings_cnt - 1) >> 3 : 0;
      for (uint8_t bytes = 0; bytes < *count; bytes++) {
	uint8_t d = 0;
	for (uint8_t bits = 0; bits < 8; bits++) {
	  d = (d << 1) | (timings[(bytes * 8) + bits + 1] < 50 ? 1 : 0);
	}
	data[bytes] = d;
      }
    } else {
      usleep (2000);
      // No, we timed out.
    } 
  } else {
    _adb_sm_state = ADB_SM_SRQ;
  }
  
  mutex_unlock(&_adb_mutex);
  _adb_sm_state = ADB_SM_IDLE;
}
  

  
