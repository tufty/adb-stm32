#include "util.h"
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/stm32/f1/rcc.h>

#pragma GCC push_options
#pragma GCC optimize ("O3")
void usleep (uint32_t usecs) {
	volatile uint32_t cycles = (rcc_ahb_frequency/1000000L)*usecs;
	volatile uint32_t start = DWT_CYCCNT;
	do  {
	} while(DWT_CYCCNT - start < cycles);
}
#pragma GCC pop_options

