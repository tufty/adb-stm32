BINARY = main
STD = -std=gnu99

CFLAGS += -I ../common

OBJS += ../common/util.o
OBJS += ../common/adb.o
OBJS += ../common/usb.o
OBJS += ../common/StrPrintf.o

LDSCRIPT = ../stm32-maplemini.ld

OPENCM3_DIR = ../libopencm3

include ../Makefile.include

DFU_UTIL = dfu-util
DFU_DEVICE = 1eaf:003
DFU_ALT = 2

dfu-upload: $(BINARY).bin
	$(DFU_UTIL) --device $(DFU_DEVICE) --alt $(DFU_ALT) -D $(BINARY).bin
