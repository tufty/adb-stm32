#include "adb.h"

int main (void) {

  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  rcc_periph_clock_enable (_rcc_adb_timer);
  rcc_periph_clock_enable (_rcc_adb_gpio);
  rcc_periph_clock_enable (_rcc_adb_dma);


  return 0;
}
