/****************************************************************************/
/*                                                                          */
/*   Copyright (c) 2006 Tim Crawford     <timcrawford@comcast.net>          */
/*                                                                          */
/*   This program is free software; you can redistribute it and/or modify   */
/*   it under the terms of the GNU General Public License version 2 as      */
/*   published by the Free Software Foundation.                             */
/*                                                                          */
/*   Alternatively, this software may be distributed under the terms of BSD */
/*   license.                                                               */
/*                                                                          */
/*   See README and COPYING for more details.                               */
/*                                                                          */
/****************************************************************************/
/*   This program demonstrates how to program the GPIO                      */
/*                                                                          */
/*   gpio(u32 direction, u32 set_clear, u32 gpio_bit)                       */
/*   example:                                                               */
/*     gpio(OUT, SET, 59); //This will program GPIO(59) as a GPIO function  */
/*                         // set the direction as output, and set the bit  */
/*                                                                          */
/*   gpio_function(u32 gpio_bit, u32 function);                             */
/*   example:                                                               */
/*     gpio_function(59, GPIO); //This will program GPIO(59) as a GPIO funct*/
/*                                                                          */
/*   gpio_direction(u32 gpio_bit, u32 direction)                            */
/*   example:                                                               */
/*     gpio_direction(59, OUT); //This will program GPIO(59) as an output   */
/*                                                                          */
/*   gpio_set(u32 gpio_bit)                                                 */
/*   example:                                                               */
/*     gpio_set(59); //This will program GPIO(59) to a logic 1              */
/*                                                                          */
/*   gpio_clear(u32 gpio_bit)                                               */
/*   example:                                                               */
/*     gpio_clear(59); //This will program GPIO(59) to a logic 0            */
/*                                                                          */
/*   gpio_status(u32 gpio_bit)                                              */
/*   example:                                                               */
/*     i = gpio_status(59); //i is set to the logic level of GPIO(59)       */
/*                                                                          */
/****************************************************************************/
#include <stdio.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <ctype.h>
#include "gpio.h"
#include "spi.h"


void *map, *regaddr;

void gpio_init(void)
{
	map = (void *)PXm_map;
	gpio_function(58, GPIO);
    gpio_function(59, GPIO);
    gpio_function(60, GPIO);

	gpio_function(66, GPIO);
    gpio_function(67, GPIO);
    gpio_function(68, GPIO);

    gpio_direction(58, OUT);
    gpio_direction(59, OUT);
    gpio_direction(60, OUT);

    gpio_direction(66, IN);
    gpio_direction(67, IN);
    gpio_direction(68, IN);

}

static void putmem(u32 addr, u32 val)
{
	//printf("in putmem\n");
   regaddr = (void*)((u32)map + (addr & MAP_MASK));
   //printf("REGADDR putmem: %lf\n", (long)regaddr);
   *(u32*) regaddr = val;
	//printf("out of putmem\n");
}
static int getmem(u32 addr)
{
	//printf("in getmem\n");
    u32 val;

    regaddr = (void*)(((u32)map) + (addr & MAP_MASK));
	//printf("REGADDR getmem: %p\n", regaddr);
    val = *(u32*) regaddr;
	//printf("out of getmem\n");
    return val;
}
void gpio_set(u32 gpio)
{
    u32 pos;
    u32 bit = 1;

    pos = gpio / 32;
    bit <<= gpio % 32;
    putmem(GPSR0 + (pos * 4), bit);
}
void gpio_clear(u32 gpio)
{
    u32 pos;
    u32 bit = 1;

    pos = gpio / 32;
    bit <<= gpio % 32;
    putmem(GPCR0 + (pos * 4), bit);
}
u32 gpio_status(u32 gpio)
{
    u32 pos;
    u32 bit = 1;
    u32 data;

    pos = gpio / 32;
    bit <<= gpio % 32;
    data = getmem(GPLR0 + (pos * 4));
    data &= bit;
    if (data == 0)
      return(0);
    else
      return(1);
}
void gpio_direction(u32 gpio, u32 dir)
{
    u32 pos;
    u32 bit = 1;
    u32 data;

    pos = gpio / 32;
    bit <<= gpio % 32;
    data = getmem(GPDR0 + (pos * 4));
    data &= ~bit;
    if (dir == OUT)
      data |= bit;
    putmem(GPDR0 + (pos * 4), data);
}
void gpio_function(u32 gpio, u32 fun)
{
    u32 pos;
    u32 bit = 3;
    u32 data;

    pos = gpio / 16;
    bit <<= (gpio % 16) * 2;
    fun <<= (gpio % 16) * 2;
    data = getmem(GAFR0_L + (pos * 4));
    data &= ~bit;
    data |= fun;
    putmem(GAFR0_L + (pos * 4), data);
}
u32 gpio(u32 dir, u32 set, u32 reg)
{
    if ((dir != IN) & (dir != OUT)){
      printf("ERROR: must specify a valid direction\n");
      return(1);
    }
    if ((set != SET) & (set != CLEAR)){
      printf("ERROR: must specify a valid level\n");
      return(1);
    }
    if (reg > 84){
      printf("ERROR: not a valid register -->%d\n", reg);
      return(1);
    }
    gpio_function(reg, GPIO);
    gpio_direction(reg, dir);
    if (dir == OUT){
      if (set == SET)
        gpio_set(reg);
      else
        gpio_clear(reg);
    }
    return(0);
}



void gpioSelChip(u8 SelChip)
{
	if(SelChip == GPIO_CHIP_1)
	{
		gpio_clear(58);
		gpio_set(59);
		gpio_set(60);
	}
	else
	if(SelChip == GPIO_CHIP_2)
	{
		gpio_set(58);
		gpio_clear(59);
		gpio_set(60);
	}
	else
	{
		gpio_set(58);
		gpio_set(59);
		gpio_clear(60);
	}
}

u32 gpioGetChip(u8 SelChip)
{
	//printf("get_chip\n");
	u32 data;	
	if(SelChip == GPIO_CHIP_1)
	{
		data = gpio_status(66);		
	}
	else
	if(SelChip == GPIO_CHIP_2)
	{
		data = gpio_status(67);		
	}
	else
	{
		data = gpio_status(68);		
	}
	//printf("done with get_chip\n");
	return data;
}

