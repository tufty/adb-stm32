#include <stdint.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencmsis/core_cm3.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/sync.h>
#include <usb.h>

// DMA enabled transition captures for signal analysis
// We will set up the timer to start on a given transition
// lo-hi or hi-lo.  Use two channels to capture the pair of
// transitions at a user-defined level, on the second transition
// we will trigger and reset the timer.
// DMA is kicked off by timer trigger, which means we need to use
// TIM1, *TIM3* or TIM5
// On timer runout (no transitions for 65536 ticks), max captures, or user
// interaction, we will stop.

mutex_t _mutex = MUTEX_UNLOCKED;

// Buffer for 1024 cptures
uint16_t captures[2048];
uint16_t capture_count = 0;
uint8_t stop_reason = 0;

const uint8_t STOP_TIMEOUT = 1;
const uint8_t STOP_OVERCAPTURE = 2;
const uint8_t STOP_USER = 4;

char * banner = "Signal monitor\r\n(c) 2019 Simon Stapleton\r\nWaiting for trigger\r\n";
bool trigger = false; /* true = trigger on rising edge */

static void wait_for_trigger (void) {
  mutex_lock            (&_mutex);
  
  dma_disable_channel   (DMA1, DMA_CHANNEL6);
  timer_disable_counter (TIM3);
  gpio_clear            (GPIOB, GPIO1);

  capture_count         = 0;
  stop_reason           = 0;
  
  exti_select_source    (EXTI4, GPIOB);
  exti_set_trigger      (EXTI4, EXTI_TRIGGER_FALLING);
  exti_enable_request   (EXTI4);
}

static void stop_capture (void) {
  timer_disable_counter (TIM3);
  dma_disable_channel   (DMA1, DMA_CHANNEL6);

  capture_count = 2048 - DMA1_CNDTR6;

  //  if (capture_count & 1) {
    //captures[capture_count++] = 0xfffe;
  //}
  
  mutex_unlock          (&_mutex);
}

void exti4_isr (void) {
  /* Disable EXTI and reset the interrupt request */
  exti_reset_request   (EXTI4);
  exti_disable_request (EXTI4);
  gpio_set(GPIOB, GPIO1);

  TIM3_DIER            = 0;
  /* Enable Timer and DMA, kick it all off! */
  TIM3_CNT             = 0;
  // Maximum transfers 2048
  DMA1_CNDTR6          = 2048;
  // Use the DMAR register as source, thus doing burst mode DMA
  DMA1_CPAR6           = (uint32_t)&TIM3_DMAR;
  DMA1_CMAR6           = (uint32_t)&(captures[0]);

  TIM3_SR = 0;
  dma_clear_interrupt_flags (DMA1, DMA_CHANNEL6, DMA_GIF);
  
  TIM3_DIER            = TIM_DIER_TDE | TIM_DIER_CC3IE;
  
  dma_enable_channel   (DMA1, DMA_CHANNEL6);
  timer_enable_counter (TIM3);
}

void dma1_channel6_isr (void) {
  dma_clear_interrupt_flags (DMA1, DMA_CHANNEL6, DMA_TCIF);
  stop_capture();
  stop_reason = STOP_OVERCAPTURE;
}

void tim3_isr (void) {
  /* if (TIM3_SR & TIM_SR_CC1IF) { */
  /*   captures[capture_count++] = TIM3_CCR1; */
  /*   timer_clear_flag (TIM3, TIM_SR_CC1IF); */
  /* } */
  /* if (TIM3_SR & TIM_SR_CC2IF) { */
  /*   captures[capture_count++] = TIM3_CCR2; */
  /*   timer_clear_flag (TIM3, TIM_SR_CC2IF); */
  /* } */
  if (TIM3_SR & TIM_SR_CC3IF) {
    timer_clear_flag (TIM3, TIM_SR_CC3IF);
    stop_capture();
    stop_reason = STOP_TIMEOUT;
  }
}

int main (void) {
  // Set up the various bits we need.
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  rcc_set_usbpre (RCC_CFGR_USBPRE_PLL_CLK_DIV1_5);

  // Enable clocking for our bits
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_TIM3);
  rcc_periph_clock_enable(RCC_DMA1);

  // Enable IRQs
  nvic_enable_irq(NVIC_TIM3_IRQ);
  nvic_enable_irq(NVIC_DMA1_CHANNEL6_IRQ);
  nvic_enable_irq(NVIC_EXTI4_IRQ);

  // Kick off USB.
  usb_vcp_init();

  /* Remap Timer3 inputs to PB 4 / 5 / 0 / 1.  First turn off JTAG */
  gpio_primary_remap  (AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, AFIO_MAPR_TIM3_REMAP_NO_REMAP); 
  gpio_primary_remap  (AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, AFIO_MAPR_TIM3_REMAP_PARTIAL_REMAP);
  
  /* Set PB4 to input, so we can capture from it */
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO4);
  // Set PB4 pull up
  GPIOB_ODR |= GPIO4;
  /* set LED to output, use it to signal capturing */
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
  gpio_clear(GPIOB, GPIO1);

  // Do basic setup of timer 3.
   // Timer counts up, internal clock, preload enabled
  TIM3_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_ARPE | TIM_CR1_DIR_UP;
  TIM3_CR2 = 0;

  // Slave mode trigger on channel 1 falling edge, trigger generates reset 
  TIM3_SMCR = TIM_SMCR_ETP | TIM_SMCR_TS_TI1FP1 | TIM_SMCR_SMS_RM;

  // enable trigger DMA transfer and interrupt on capture-compare channel 3
  TIM3_DIER = TIM_DIER_TDE | TIM_DIER_CC3IE;
  // Disable all capture-compares so we can mess with the settings
  TIM3_CCER = 0;
  // CC1 & CC2 set up to capture PWM on TI1 with a x4 filter (4 µsec delay)
  TIM3_CCMR1 = TIM_CCMR1_CC2S_IN_TI1 | TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_CK_INT_N_4 | TIM_CCMR1_IC2F_CK_INT_N_4;
  // CC3 will count up to the value in CCR3, preload and fast enabled, no output to its pin
  TIM3_CCMR2 = TIM_CCMR2_OC3M_FROZEN | TIM_CCMR2_OC3PE | TIM_CCMR2_OC3FE | TIM_CCMR2_CC3S_OUT;
  // enable the 3 capture ompare channels, invert polarity on CC1
  TIM3_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC2P;
  // Reset the counter
  TIM3_CNT = 0;
  // Prescaler to 71, 72MHz clock so 1µsec "tick"
  TIM3_PSC = 71;
  // Set the period to something we'll never reach
  TIM3_ARR = 0xffff;
  TIM3_CCR3 = 0xfffe;
  // Set up dma for bursts of 2, CC1 & CC2, which is where our bit timing data should reside
  TIM3_DCR = ((2 - 1) << 8) | (0x34 >> 2);
  TIM3_DMAR = 0;

  // Set up DMA to do 16 bit transfers, incrementing the memory address every time, interrupt on transfer complete, priority high
  DMA1_CCR6 = DMA_CCR_PL_HIGH | DMA_CCR_MSIZE_16BIT | DMA_CCR_PSIZE_16BIT | DMA_CCR_MINC | DMA_CCR_TCIE;

    
  while (1) {
    // Wait for connect
    while (!usb_vcp_is_connected()) __WFI();
    
    usb_vcp_printf(banner);

    for (int i = 0; i < 2048; i++) captures[i] = 0xffff;

    wait_for_trigger();

    while ((_mutex == MUTEX_LOCKED) && (usb_vcp_avail() == 0)) {
      __WFI();
    }

    stop_capture();
    while (usb_vcp_avail() != 0) usb_vcp_recv_byte();

    gpio_clear(GPIOB, GPIO1);

    usb_vcp_printf("Capture stopped due to %s\n", stop_reason == 0 ? "user keypress" : stop_reason == STOP_OVERCAPTURE ? "overcapture" : "timeout");
    usb_vcp_printf("Captured %u samples\n", capture_count); 

    for (int i = 0; i < capture_count; i += 2) {
      usb_vcp_printf("%u, %u\n", captures[i], captures[i+1]);
    }

  }
}

  
