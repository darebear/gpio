/****************************************************************************
*
*   Copyright (c) 2006 Dave Hylands     <dhylands@gmail.com>
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation.
*
*   Alternatively, this software may be distributed under the terms of BSD
*   license.
*
*   See README and COPYING for more details.
*
****************************************************************************
*
*   User mode app which monitors multiple gpio pins and notifies a user mode
*   app about the events.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <ctype.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Additional includes from main.cpp
#include <stdint.h>
//#include <sys/types.h>
#include <signal.h>
//#include <list>

// gpio-notify
#include <sys/ioctl.h>

// for geiger handling
#include <pthread.h>
#include <signal.h>

//#include "svn-version.h"

#if !defined( SVN_REVISION )
#   define SVN_REVISION  0
#endif

#include "gpio-event-drv.h"

#define GPIO_DEVICE_FILENAME "/dev/gpio-geiger"

/***************************************************************************
* Geiger handler function  
****************************************************************************/
sigset_t mask;

void* geiger_handler(void *arg){
	//char tmp_buf[1024], temp1;
	//int signo, i, j;
	int signo;
    //int inputId = *(int *)arg;
    
	for(;;){
        
        // Suspend geiger_handler thread until a signal is available
        //
        printf("At sigwait.. \n");
		sigwait(&mask, &signo); //&mask is defined in main function
        printf("Sigwait triggered! \n");

        
        /*
		int n = ((RS232 *)comm)->rx( (char *)tmp_buf, 1024);
		//printf("enqueue_incoming:%d\n", n);
		//if(n > (CMD_PKT_SZ+3) || n < CMD_PKT_SZ) continue;
		if(n < RCV_PKT_SZ) continue;
		for(i=0, j=0; i<n; i++){
			//readBuffer.push_back(tmp_buf[i]);
			temp1 = tmp_buf[i];
			//if(i == 0) printf("enqueue_incoming (%d): ", n);
			printf("[%x]", temp1);
			if(temp1 == CMD_START || temp1 == ACK_START || temp1 == DATA_START || j > 0);
			else continue;

			//cout << "Enqueue_Incoming start..." << endl;
			rcv_buff[j] = temp1; j++;
			if(temp1 == CMD_END || temp1 == ACK_END || temp1 == DATA_END) 
			{
				//printf("[%d:%d]\n", i, temp1);
				printf("\n");
				temp1 = rcv_buff[RCV_PKT_SZ-1];
				if((rcv_buff[0] == CMD_START && temp1 == CMD_END)
				|| (rcv_buff[0] == ACK_START && temp1 == ACK_END)
				|| (rcv_buff[0] == DATA_START && temp1 == DATA_END))
					queue_enqueue(&rx_queue,rcv_buff);
				else printf("RS232 messages are not accepted!!\n");
				break;
			}

		}
        */
		//if(i > 0) printf("\n");
	}
} // geiger_thread

/****************************************************************************
*
*  main
*
***************************************************************************/

int main( int argc, char **argv )
{

    FILE               *fs;

    if (( fs = fopen( "/dev/gpio-geiger", "r" )) == NULL )
    {
        perror( "Check to make sure gpio_event_drv has been loaded. Unable to open /dev/gpio-event" );
        exit( 1 );
    }
    

    /*
    // Multi-threading
	// Set SIGIO such that the main process ignores the signal
    // POSIX signal set
	sigset_t oldmask;
    //Initializes the signal set given by &mask to empty w/all signals excluded from the set
	sigemptyset(&mask);
    //Add SIGIO signal to &mask
	sigaddset(&mask, SIGIO);    // SIGIO is a signat that indicates an input/output status change 
    // SIG_BLOCK creates union of &mask and the current signal set, however at this point &mask is just SIGIO.
    // &mask is copied over to &oldmask 
	pthread_sigmask(SIG_BLOCK, &mask, &oldmask);

	// Setup Read Geiger Thread
	pthread_t geiger_thread;
	pthread_create(&geiger_thread, NULL, geiger_handler, NULL); 
	fcntl( fileno(fs), F_SETOWN, geiger_thread);
	//fcntl( fileno(fs), F_SETOWN, getpid() );
	fcntl( fileno(fs), F_SETFL, FASYNC);
    // End Read Geiger Thread Setup
    printf("Set FASYNC flag and created geiger_thread thread\n");
    */

    // Set up the GPIO
    GPIO_EventMonitor_t     monitor;  // defined in gpio_event_driver.h 
    monitor.gpio    = 168;
    monitor.onOff   = 1;
    monitor.edgeType = GPIO_EventRisingEdge;
    monitor.debounceMilliSec = 0;
    
    printf("At ioctl fileno check.\n");
    if ( ioctl( fileno( fs ), GPIO_EVENT_IOCTL_MONITOR_GPIO, &monitor ) != 0 )
    {
        perror( "ioctl GPIO_EVENT_IOCTL_MONITOR_GPIO failed" );
    }
    
    // Manual reading of file
    ssize_t numBytes;
    char    argStr[ 60 ];
    argStr[ 0 ] = '\0';

    int gMonitor = 1;
    int counter = 0;

    while(1)
    {
        printf(" at fgets.. \n");
        // fgets stores incoming bytes from fs into argStr until an EOL or EOF has been read
        if ( fgets( argStr, sizeof( argStr ), fs ) != NULL )
            {
                counter++;
                numBytes = strlen( argStr );
                printf(" counter = %d \n", counter);

                if ( argStr[ numBytes - 1 ] == '\n' )
                {
                    argStr[ numBytes - 1 ] = '\0';
                }

                if ( gMonitor )
                {
                    printf( "Read:/ '%s'\n", argStr );
                }
            }
        else
        {
            argStr[ 0 ] = '\0';
        }
    }

    printf( "closing app \n");
    //fclose( fs );
    //exit( 0 );
    return 0;

} // main
