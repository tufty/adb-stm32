#include <util.h>
#include <adb.h>
#include <usb.h>
#include <libopencmsis/core_cm3.h>
#include <libopencm3/cm3/dwt.h>

int main (void) {

  // Set up clock and usb prescaler.
  rcc_clock_setup_in_hse_8mhz_out_72mhz ();
  rcc_set_usbpre (RCC_CFGR_USBPRE_PLL_CLK_DIV1_5);

  rcc_periph_clock_enable (RCC_TIM2);
  rcc_periph_clock_enable (RCC_GPIOA);
  rcc_periph_clock_enable (RCC_GPIOB);
  rcc_periph_clock_enable (RCC_AFIO);
  rcc_periph_clock_enable (RCC_DMA1);
  rcc_periph_clock_enable (RCC_OTGFS);

  dwt_enable_cycle_counter();

  usb_vcp_init();
  
  while (1) {
    usleep (1000000);
    usb_vcp_send_strn("Hello from ADBTerm\r\n", 20); 
    __WFI();
  }
}
