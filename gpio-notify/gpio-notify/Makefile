#############################################################################
#
#	Makefile for building the char-driver sample module
#
#############################################################################

ifeq ($(DEBUG),y)
	CFLAGS += -O -g		# -O is need to expand inlines
else
	CFLAGS += -O2
endif

ifneq ($(KERNELRELEASE),)
# call from kernel build system

obj-m	:= gpio-notify-drv.o

else

PWD       := $(shell pwd)

#GUMSTIX_BUILDROOT	= $(PWD)/../cx-1575
#BUILD_ARM		= $(wildcard $(GUMSTIX_BUILDROOT)/build_arm*)
#KERNELDIR 		?= $(wildcard $(BUILD_ARM)/linux-*)
#CROSS_COMPILE		= $(BUILD_ARM)/staging_dir/bin/arm-linux-

default: user-apps kernel-module

.PHONY: user-apps kernel-module

user-apps: gpio-notify

TARGET_ARCH=-Os -march=armv5te -mtune=xscale -Wa,-mcpu=xscale
#CC = $(CROSS_COMPILE)gcc

CPPFLAGS	+= 
CFLAGS		+= -Wall -lpthread

GPIO_HEADERS := gpio.h spi.h dcb.h
GPIO_SOURCES := gpio.c spi.c

gpio-notify: gpio-notify.c $(GPIO_SOURCES) $(GPIO_HEADERS)
	$(CC) $(CFLAGS) -o $@ $< $(GPIO_SOURCES)

kernel-module:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) modules

endif

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.symvers
	rm -rf gpio-notify

depend .depend dep: #svn-version.h
	@echo "Creating dependencies ..."
	$(CC) $(CFLAGS) -M *.c $(SHARED)/*.c > .depend

#include $(ROBOSTIX)/svn-version.mk
								 
ifeq (.depend,$(wildcard .depend))
include .depend
endif
