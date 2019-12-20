#ifndef __CDCACM_H__
#define __CDCACM_H__

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

extern void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue);

#endif
