/************************************************************************** 
 *       SPI Sample code.  (Direct Regsiter Access via /dev/mem)          *
 *                                                                        *
 *  This code is fairly straight-forward and does not require extra       *
 *  kernel modules to obtain SPI communication via the NSSP Pins          *
 *  on the GumStix.  Comments have been included for alternate            *
 *  bit configurations where appropriate.  The pxa255 supports            *
 *  Motorola SPI, TexasInstruments SSP, and Microwire.                    *
 *                                                                        *
 *  See the "SPIsetConReg()" function below for more information.         *
 *  This example is currently set for:                                    *
 *                           1.6Mbit/sec, SPI, 8-bit-data, idle=low       * 
 *  GPIO Pins:                                                            * 
 *        81 to NSSPClk                                                   *     
 *        82 to NSSPSFRM                                                  *
 *        83 to NSSPTXD                                                   *
 *        84 to NSSPRXD                                                   *
 *  Enjoy!                                                                *
 *                                                                        *
 **************************************************************************/

/**************************************************************************
 *  Created by: Ernest Earon                                              *
 * Modified by: Adam Kumpf - kumpf@mit.edu (July 2006)                    *
 *                                                                        *
 * This program is free software; you can redistribute it and/or modify   *
 * it under the terms of the GNU General Public License as published by   *
 * the Free Software Foundation; either version 2 of the License, or      *
 * (at your option) any later version.                                    *
 *                                                                        *
 * This program is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 * GNU General Public License for more details.                           *
 *                                                                        *
 * You should have received a copy of the GNU General Public License      *
 * along with this program; if not, write to the Free Software            *
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.              *
 **************************************************************************/

// ---- Include Files ----------------------------------------------------
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <stdio.h>
#include "spi.h"



//----------------------------------------------------------------
// ---- Global Variables -----------------------------------------
//----------------------------------------------------------------
unsigned long int* PXm_map;
unsigned long int* CR_map;

int gpio_fd;

//***************************************************************************

//***************************************************************************
/**
*
*   Main
*/
/*
int main(int argc, char *argv[])
{
	long i = 0;

		// SPI Datarate Settings...
		// ClockDivider = 0  ~ 1.62 Mbit/sec 	
		// ClockDivider = 4  ~  480 Kbit/sec
		// ClockDivider = 8  ~  300 Kbit/sec
		// ClockDivider = 16 ~  170 Kbit/sec
	int ClockDivider = 0;	
	
	
	// start the main routine
	printf("\n\n");
	printf("   ------------------------------------------\n");
	printf("   |               SPI Sample                |\n");
	printf("   ------------------------------------------\n");
	printf("   |   Direct Register Access SPI Sample     |\n");
	printf("   ------------------------------------------\n");

	printf(" -> Msg<- Initializing SPI port\n");
	int result = SPIinit(ClockDivider);	// Initialize NSSP / SPI Port
	printf(" -> Msg<- Result of SPI Init = %d\n",result);
	
	printf(" -> Msg<- Sending test characters....\n");
	//for(i=0; i<0x0200; i++){
	while(1){
		
		int rx = SPI_TxRx(i & 0xffff); 	// Transmit/Receive via NSSP/SPI port
		printf("0x%04x(0x%04x) \n",i & 0xffff,rx);
		fflush(stdout);
		usleep(40000);
	}
	printf("\n");

	printf(" -> Msg<- Closing NSSP/SPI Port\n");
	close(gpio_fd);
	printf(" -> Msg<- done.\n\n");
	
    return 1;
}

*/

//***************************************************************************
/**
*
*   SPI Initialization
*/
int SPIinit (int ClockDivider)
{
        gpio_fd = open("/dev/mem",O_RDWR | O_SYNC);

        if (gpio_fd < 0)
                return -1;

        PXm_map = (unsigned long int*) mmap(NULL, 4096UL, 
        		PROT_READ | PROT_WRITE, MAP_SHARED, gpio_fd, GPIO_BASE_OFFSET);
		//printf("PXm_map value: %p\n", PXm_map);
        if (PXm_map <= 0)
                return -2;

        SPIsetDir();		// set the pin directions for 81,82,83,84
        
        SPIsetFunc();		// set the alternate functions of pins 81,82,83,84:
        
        return SPIsetConReg(ClockDivider);  // Set the nssp function to support motorola SPI
}

//***************************************************************************
/**
*
*   SPI Transmit / Receive. (they happen at the same time)
*/
int SPI_TxRx(int b) 	// send and receive a byte through the SPI
{
        unsigned long int* volatile Reg_FIFO;
        unsigned long int* volatile Reg_Status;
	int q;


    	Reg_FIFO = (unsigned long int *)(CR_map + NSSDR);               
    	Reg_Status = (unsigned long int *)(CR_map + NSSSR);               

	q = (*Reg_FIFO) & 0xffff;	//
	q = (*Reg_FIFO) & 0xffff;	//

    	while(((*Reg_Status) & 32) == 0); 	// wait for TX FIFO to be at/below threshold 
      	while((*Reg_Status) & 16);		// wait for SSP to not be Busy

  	*Reg_FIFO = (unsigned long int)b; 	// Set the Tx Value
	
	for(q=0;q<50;q++); // just a tiny delay so Tx will start to go...
	
    	while(((*Reg_Status) & 64) == 0); 	// wait for RX FIFO to be at/above threshold 
      	while((*Reg_Status) & 16);		// wait for SSP to not be Busy

        return (*Reg_FIFO) & 0xffff; 	// return the Rx Value
}


//***************************************************************************
/**
*
*   SPI Configuration Setup:  EDIT THIS!!  :)
*
*   0 <= ClockDivider <= 255
*   BitRate = 3.6864x106 / (2 x (ClockDivider + 1))
*/
int SPIsetConReg (int ClockDivider)	// set the control registers for SPI
{
        int gpio_fd = open("/dev/mem",O_RDWR | O_SYNC);
        unsigned long int* volatile Reg;
	
	ClockDivider = ClockDivider & 0xff;
	
        if (gpio_fd < 0)
                return -1;

        CR_map = (unsigned long int*) mmap(NULL, 4096UL, 
        			PROT_READ | PROT_WRITE, MAP_SHARED, gpio_fd, NSSPCR0);

        if (CR_map <= 0)
                return -2;

        Reg = (unsigned long int *)(CR_map + NSSPCR1);
        (*Reg) = 0x40000000;
        	// -----------CR1 Bit Definitions ------------
        	// bit 5 = Microwire TxSize (0=8-bit,1=16-bit);
        	// bit 4 = Clock Phase (0=1:0.5, 1=0.5:1)       
        	// bit 3 = Clock Polarity Setting (0=idle_is_low, 1=idle_is_high)   
        //*Reg |= 1 << 5;  // example, this is how you would set bit 5 for 16-bit Microwire


        Reg = (unsigned long int *)(CR_map + 0); // this will also enable the SPI
        	// -----------CR0 Bit Definitions ------------
        	// bit 15:8 = clockrate divider
        	// bit    7 = SSP Enable
        	// bit    6 = Select External Clock (0=internal)
        	// bit  5:4 = frame format (0=Motorola, 1=TexInst, 2=Natl.Microwire)
        	// bit  3:0 = data bits - 1 (ex. 0x7 = 8-data-bits, 0xf = 16-data-bits)
      
        *Reg  = 0x07; 	// 16-data-bits,Motorola format (use 0x07 for 8-data-bits,Motorola)
        *Reg |= ClockDivider << 8; 
        *Reg |= 1 << 7;	// Enable SPI
        printf(" -> Msg<- SPIsetConReg: CR0 = 0x%04x\n",(*Reg));
        return 1;
}

void SPIsetFunc (void)
{
        // map offset:
        // pins 81-84 are in 6th AF reg (AFR 5)
        // byte position is 5 * bytes per unit + AF Reg Offset = 5 * 4 + 0x54 = 104
        // 104 bytes to words = 104/4 = 26        --> 26 <--
        unsigned long int* volatile pGPAF = (PXm_map + 26);
        //Set 	81 to NSSPClk               
        //     	82 to NSSPSFRM
        //     	83 to NSSPTXD
        //     	84 to NSSPRXD
        // What pins do we set to 1s?

        // what pins do we set to 0s?
        *pGPAF = 596;
}

void SPIsetDir (void)
{
        // map offset:
        // pins 81-84 are in 3rd Dir reg (DirR 2)
        // byte position is 2 * bytes per unit + Dir Reg Offset = 2 * 4 + 0xC = 20
        // 104 bytes to words = 20/4 = 5        --> 5 <--
        unsigned long int* volatile pGPD = (PXm_map + 5);
        //Set  	81 to out      	bit 17     	1
        //    	82 to out    	    18         	1
        //     	83 to out  	    19       	1
        //     	84 to in      	    20       	0
        *pGPD = 0xE0000;
}
