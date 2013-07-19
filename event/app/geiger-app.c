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
#include <signal.h>

// gpio-notify
#include <sys/ioctl.h>
#include <sys/syscall.h>

// for geiger handling
#include <pthread.h>
#include <signal.h>

#include "gpio-event-drv.h"

#define GPIO_DEVICE_FILENAME "/dev/gpio-event"
#define GPIO_GEIGER_POWER 147 
#define GPIO_GEIGER_SIGNAL 146 

/***************************************************************************
* Geiger handler function  
****************************************************************************/
sigset_t mask;

void* geiger_handler(void *arg){
	//char tmp_buf[1024], temp1;
	//int signo, i, j;
	int signo;
    int counter=0;
    //int inputId = *(int *)arg;
    
	for(;;){
        
        // Suspend geiger_handler thread until a signal is available
		sigwait(&mask, &signo); //&mask is defined in main function
        counter++;
        printf("Counter %d \n", counter); 

 //       printf("Sigwait triggered! \n");

        
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


int geiger_counter_power(int onOff){
    
    int rc = 0;
    int power_pin = 19;
    /*
    if (onOff == 0){
        printf("Turning geiger counter on \n");
        system("echo 19 > /sys/class/gpio/export");
        system("echo out > /sys/class/gpio/gpio19/direction");
        system("echo 1 > /sys/class/gpio/gpio19/value");
    }
    else{
        printf("Turning geiger counter off \n");
        system("echo 19 > /sys/class/gpio/export");
        system("echo out > /sys/class/gpio/gpio19/direction");
        system("echo 0 > /sys/class/gpio/gpio19/value");
    }
    */

    if (onOff == 0){
        printf("Turning geiger counter off \n");
        system("sh powerGpio.sh 0 147");
    }
    else{
        printf("Turning geiger counter on \n");
        system("sh powerGpio.sh 1 147");
    }
    


    return rc;

}//end geiger_counter_power


//#define MULTITHREADING
//#define MULTITHREADING_LDD3
//#define MULTITHREADING_MAIN 
//#define REPORT_IN_MAIN 
//#define OPEN_EVENT
#define OPEN_NOTIFY
#define FASYNC_NOTIFY 


/****************************************************************************
*
*  main
*
***************************************************************************/

int main( int argc, char **argv )
{

    FILE               *fs;

#ifdef GEIGER_TESTING
    // Testing power to the geiger counter
    int i;
    for (i=0; i<10; i++){
        printf(" i = %d ... \n", i );
        geiger_counter_power(1);
        sleep(2);
        geiger_counter_power(0);
        sleep(2);
    }
#endif

    // Turn Geiger counter on for monitoring
    geiger_counter_power(1);

#ifdef OPEN_EVENT
    if (( fs = fopen( GPIO_DEVICE_FILENAME, "r" )) == NULL )
    {
        perror( "Check to make sure gpio_event_drv has been loaded. Unable to open /dev/gpio-event" );
        exit( 1 );
    }
    int fd = fileno(fs);
    printf( "Opened device file %s on fd %d. \n", GPIO_DEVICE_FILENAME, fd);
#endif

#ifdef OPEN_NOTIFY
    int fd = open( GPIO_DEVICE_FILENAME, O_RDONLY | O_NONBLOCK );
    if (fd==-1)
    {
        printf( "Could not open %s \n", GPIO_DEVICE_FILENAME);
        exit(1);
    }
    printf( "Opened device file %s on fd %d. \n", GPIO_DEVICE_FILENAME, fd);
#endif

#ifdef FASYNC_NOTIFY
    
    // specify which signals we will catch
    sigset_t oldmask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGIO);

    // Setup Read Geiger Thread
	pthread_sigmask(SIG_BLOCK, &mask, &oldmask);
	pthread_t geiger_thread;
	pthread_create(&geiger_thread, NULL, geiger_handler, NULL); 
    // Set file ownership to the geiger thread
	//fcntl( fd, F_SETOWN, geiger_thread);
    //TODO: why does it work with getpid() but not assigning to the geiger_thread?
	int owner_id = fcntl( fd, F_SETOWN, getpid());
    printf("The process id owner of %s is %d \n", GPIO_DEVICE_FILENAME, owner_id);
	fcntl( fd, F_SETOWN, getpid());
    //printf("The new process id owner of %s is %d \n", GPIO_DEVICE_FILENAME, owner_id);
	fcntl( fd, F_SETFL, FASYNC);
    //int oflags = fcntl( fd, F_GETFL);
    //fcntl( fd, F_SETFL, oflags | FASYNC);
    printf("FASYNC flag set and ownership given to the geiger thread... \n");

    /*
    // set FASYNC flag on the device file to enable SIGIO notifications
    printf("Setting FASYNC flag.\n");
    int oflags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, oflags | FASYNC);
    */

    // Set up the GPIO
    printf("Setting up the GPIO ... \n");
    GPIO_EventMonitor_t     monitor;  // defined in gpio_event_driver.h 
    monitor.gpio    = 146;
    //monitor.gpio    = 168;
    monitor.onOff   = 1;
    monitor.edgeType = GPIO_EventRisingEdge;
    monitor.debounceMilliSec = 0;
    
    printf("Enable monitoring on pin %d.\n", monitor.gpio);
    if ( ioctl( fd, GPIO_EVENT_IOCTL_MONITOR_GPIO, &monitor ) != 0 )
    {
        perror( "ioctl GPIO_EVENT_IOCTL_MONITOR_GPIO failed" );
        printf( "ioctl GPIO_EVENT_IOCTL_MONITOR_GPIO failed" );
    }
    printf("GPIO monitoring set up ...\n");
    
    //int i = 0;
    //for (i=0;40;i++)
    //{
        //printf("Main thread counter %d \n", i); 
        //sleep(1);
    //}
        
#endif


#ifdef MULTITHREADING_MAIN
    // Multi-threading
	// Set SIGIO such that the main process ignores the signal
    // POSIX signal set
	sigset_t oldmask;
    //Initializes the signal set given by &mask to empty w/all signals excluded from the set
	sigemptyset(&mask);
    //Add SIGIO signal to &mask
	sigaddset(&mask, SIGIO);    // SIGIO is a signal that indicates an input/output status change 
    // SIG_BLOCK creates union of &mask and the current signal set, however at this point &mask is just SIGIO.
    // &mask is copied over to &oldmask 
	pthread_sigmask(SIG_BLOCK, &mask, &oldmask);
#endif 

#ifdef MULTITHREADING_MAIN
	// Setup Read Geiger Thread
	pthread_t geiger_thread;
	pthread_create(&geiger_thread, NULL, geiger_handler, NULL); 
	fcntl( fd, F_SETOWN, geiger_thread);
	fcntl( fd, F_SETFL, FASYNC);
    // End Read Geiger Thread Setup
    printf("Set FASYNC flag and created geiger_thread thread\n");
#endif

#ifdef MULTITHREADING_LDD3
	pthread_t geiger_thread;
	pthread_create(&geiger_thread, NULL, geiger_handler, NULL); 
    signal(SIGIO, &geiger_handler);
    fcntl( fd, F_SETOWN, geiger_thread);
    int oflags = fcntl( fd, F_GETFL);
    fcntl( fd, F_SETFL, oflags | FASYNC);
    printf("Multithreading with LDD3 \n");


    // Set up the GPIO
    GPIO_EventMonitor_t     monitor;  // defined in gpio_event_driver.h 
    monitor.gpio    = 168;
    monitor.onOff   = 1;
    monitor.edgeType = GPIO_EventRisingEdge;
    monitor.debounceMilliSec = 0;
    
    printf("At ioctl fileno check.\n");
    if ( ioctl( fd, GPIO_EVENT_IOCTL_MONITOR_GPIO, &monitor ) != 0 )
    {
        perror( "ioctl GPIO_EVENT_IOCTL_MONITOR_GPIO failed" );
    }
    
#endif
    // Manual reading of file
    ssize_t numBytes;
    char    argStr[ 60 ];
    argStr[ 0 ] = '\0';

    int gMonitor = 1;
    int counter = 0;

    while(1)
    {
#ifdef REPORT_IN_MAIN
        printf("Reporting in main.. \n");
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
                    printf( "Read: '%s'\n", argStr );
                }
            }
        else
        {
            argStr[ 0 ] = '\0';
        }
#endif 
    }

    printf( "closing app \n");
    fclose( fs );
    exit( 0 );
    return 0;

} // main
