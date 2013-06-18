#ifndef __DCB_H
	#include"dcb.h" 
#endif
#ifndef __SPI_H
	#define __SPI_H
    
	//------------------------------------------------------------------------
	// ----  Memory Location Definitions -------------------------------------
	//------------------------------------------------------------------------
	// See the Intel(r) PXA255 Processor Developer's Manual

	#define 	GPIO_BASE_OFFSET      	0x40E00000
	#define 	SPIDirROff            	0x00000014
	#define 	SPIAfROff             	0x00000068

	// Register definitions in the PXA255 manual in byte locations
	#define        NSSPCR0_BL      	0x41400000
	#define        NSSPCR1_BL      	0x00000004
	#define        NSSSR_BL        	0x00000008
	#define        NSSITR_BL       	0x0000000C
	#define        NSSDR_BL        	0x00000010
	#define        NSST0_BL        	0x00000028
	#define        NSSPSP_BL       	0x0000002C

	// Register definitions in word locations
	#define        NSSPCR0    	0x41400000
	#define        NSSPCR1 		0x00000001
	#define        NSSSR          	0x00000002
	#define        NSSITR         	0x00000003
	#define        NSSDR          	0x00000004
	#define        NSST0           	0x0000000a
	#define        NSSPSP          	0x0000000b



	//--------------------------------------------------------------------
	// ----  Function Prototypes -----------------------------------------
	//--------------------------------------------------------------------
	extern	int    	SPIinit(int ClockDivider);   	// initialise the SPI stuff
	extern	int  	SPIsetConReg(int ClockDivider);	// set the control registers for SPI
	extern	void  	SPIsetFunc(void);   		// set alternate functions of pins to NSSP
	extern	void  	SPIsetDir(void);      		// set the appropriate directions
	extern	int 	SPI_TxRx(int b);		// Write a byte and read a byte



	//----------------------------------------------------------------
	// ---- Global Variables -----------------------------------------
	//----------------------------------------------------------------
	extern	unsigned long int* PXm_map;
	extern	unsigned long int* CR_map;

	extern	int gpio_fd;

#endif

