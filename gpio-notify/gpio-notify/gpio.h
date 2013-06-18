#ifndef __DCB_H
	#include"dcb.h" 
#endif
#ifndef __GPIO_H
	#define __GPIO_H
    
	//------------------------------------------------------------------------
	// ----  Memory Location Definitions -------------------------------------
	//------------------------------------------------------------------------
	// See the Intel(r) PXA255 Processor Developer's Manual

	#define MAP_SIZE 4096
	#define MAP_MASK ( MAP_SIZE - 1 )
	#define GPLR0   0x40E00000
	#define GPLR1   0x40E00004
	#define GPLR2   0x40E00008
	#define GPDR0   0x40E0000C
	#define GPDR1   0x40E00010
	#define GPDR2   0x40E00014
	#define GPSR0   0x40E00018
	#define GPSR1   0x40E0001C
	#define GPSR2   0x40E00020
	#define GPCR0   0x40E00024
	#define GPCR1   0x40E00028
	#define GPCR2   0x40E0002C
	#define GAFR0_L 0x40E00054
	#define GAFR0_U 0x40E00058
	#define GAFR1_L 0x40E0005C
	#define GAFR1_U 0x40E00060
	#define GAFR2_L 0x40E00064
	#define GAFR2_U 0x40E00068



	#define IN 250
	#define OUT 251
	#define GPIO 0
	#define AF0 0
	#define AF1 1
	#define AF2 2
	#define AF3 3
	#define SET 252
	#define CLEAR 253

	#define GPIO_CHIP_1  1
	#define GPIO_CHIP_2  2
	#define GPIO_CHIP_3  3



	//typedef unsigned int u32;

	//--------------------------------------------------------------------
	// ----  Function Prototypes -----------------------------------------
	//--------------------------------------------------------------------
	

	//extern	void putmem(u32 addr, u32 val);
	//extern	int getmem(u32 addr);
	extern	void gpio_init(void);
	extern	void gpio_set(u32 gpio);
	extern	void gpio_clear(u32 gpio);
	extern	u32 gpio_status(u32 gpio);
	extern	void gpio_direction(u32 gpio, u32 dir);
	extern	void gpio_function(u32 gpio, u32 fun);
	extern	u32 gpio(u32 dir, u32 set, u32 reg);
	extern	void gpioSelChip(u8 SelChip);
	extern  u32 gpioGetChip(u8 SelChip);

	//----------------------------------------------------------------
	// ---- Global Variables -----------------------------------------
	//----------------------------------------------------------------
	extern	void *map, *regaddr;

#endif

