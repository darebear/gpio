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

#FOOFOO
#OVEROTOP	?= /home/dario/overo-oe
#CROSS_COMPILE	?= $(OVEROTOP)/tmp/sysroots/i686-linux/usr/armv7a/bin/arm-angstrom-linux-gnueabi-
#DV6
#OVEROTOP	?= /home/d/yocto/build
#CROSS_COMPILE	?= $(OVEROTOP)/tmp/sysroots/x86_64-linux/usr/bin/armv7a-vfp-neon-poky-linux-gnueabi/arm-poky-linux-gnueabi-
#CROSS_COMPILE	?= $(OVEROTOP)/tmp/sysroots/x86_64-linux/usr/bin/armv7a-vfp-neon-poky-linux-gnueabi-
#ALL
CROSS_COMPILE = /usr/bin/arm-linux-gnueabi-

default: user-apps 

.PHONY: user-apps 

user-apps: geiger-app 

CC = $(CROSS_COMPILE)gcc

CPPFLAGS	+= 
#CFLAGS		+= -Wall -lpthread
CFLAGS		+= -Wall
LDFLAGS 	+= -lpthread

GPIO_HEADERS := gpio.h spi.h dcb.h
GPIO_SOURCES := gpio.c spi.c

geiger-app: geiger-app.c 
	$(CC) $(CFLAGS) -o $@ $< 

endif

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.symvers
	rm -rf geiger-app

depend .depend dep: #svn-version.h
	@echo "Creating dependencies ..."
	$(CC) $(CFLAGS) -M *.c $(SHARED)/*.c $(LDFLAGS) > .depend

ifeq (.depend,$(wildcard .depend))
include .depend
endif
