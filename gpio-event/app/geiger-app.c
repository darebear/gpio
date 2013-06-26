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

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>

// just in case #include <time.h> (main.cpp)
#include <stdint.h>
//#include <stdlib.h>
//#include <vector>
//#include <sys/types.h>
//#include <unistd.h>
//#include <pthread.h>
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

/* ---- Public Variables ------------------------------------------------- */

int     gDebug      = 0;
int     gVerbose    = 0;
int     gBinary     = 0;
int     gMonitor    = 0;
char   *gExecuteStr = NULL;
int     gSelect     = 0;

/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */

enum
{
    // Options assigned a single character code can use that charater code
    // as a short option.

    OPT_BINARY      = 'b',
    OPT_DEBUG       = 'd',
    OPT_EXECUTE     = 'e',
    OPT_MONITOR     = 'm',
    OPT_VERBOSE     = 'v',
    OPT_VERSION     = 'V',
    OPT_SELECT      = 's',

    // Options from this point onwards don't have any short option equivalents

    OPT_FIRST_LONG_OPT   = 0x80,

    OPT_HELP,
};

// option type from <getopt.h>. http://www.gnu.org/software/libc/manual/html_node/Getopt-Long-Options.html#Getopt-Long-Options 
// struct option fields: {const char *name, int has_arg, int *flag, int val}
struct option gOption[] =
{
    { "binary",     no_argument,        NULL,       OPT_BINARY },
    { "execute",    required_argument,  NULL,       OPT_EXECUTE },
    { "monitor",    no_argument,        NULL,       OPT_MONITOR },
    { "select",     no_argument,        NULL,       OPT_SELECT },
    { "version",    no_argument,        NULL,       OPT_VERSION },
    { "verbose",    no_argument,        NULL,       OPT_VERBOSE },
    { "debug",      no_argument,        NULL,       OPT_DEBUG },
    { "help",       no_argument,        NULL,       OPT_HELP },
    { NULL }
};


/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

//***************************************************************************
/**
*   Usage
*/

static void Usage( void )
{
    fprintf( stderr, "Usage: gpio-event [options] gpio[:edge[:debounce]] ...\n" );
    fprintf( stderr, "  where edge can be rising, falling, both, or r, f, b\n" );
    fprintf( stderr, "  -b, --binary        Use the binary interface\n" );
    fprintf( stderr, "  -e, --execute CMD   Execute the given program\n" );
    fprintf( stderr, "  -m, --monitor       Monitor the gpio events\n" );
    fprintf( stderr, "  -s, --select        Wait for event using select\n" );
    fprintf( stderr, "  -V. --version       Print the version information for this program\n" );
    fprintf( stderr, "  -v, --verbose       Print additional information\n" );
    fprintf( stderr, "  -h, --help          Prints this information\n" );

} // Usage

/***************************************************************************
* Geiger handler function  
****************************************************************************/
sigset_t mask;
//void* geiger_handler_function(void * comm){
void* geiger_handler_function(void *arg){
	//char tmp_buf[1024], temp1;
	//int signo, i, j;
	int signo;
    int inputId = *(int *)arg;
    
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
} // geiger_handler

/****************************************************************************
*
*  main
*
***************************************************************************/

int main( int argc, char **argv )
{
    char                shortOptionString[ sizeof( gOption ) / sizeof( gOption[ 0 ] ) + 1 ];
    char               *shortOpts = shortOptionString;
    struct option      *scanOpt;
    int                 opt;
    int                 arg;
    FILE               *fs;

    // Figure out the short options from our options structure

    for ( scanOpt = gOption; scanOpt->name != NULL; scanOpt++ ) 
    {
        if (( scanOpt->flag == NULL ) && ( scanOpt->val < OPT_FIRST_LONG_OPT )) //there are short options
        {
            *shortOpts++ = (char)scanOpt->val;

            if ( scanOpt->has_arg != no_argument )
            {
                *shortOpts++ = ':';
            }
        }
    }
    *shortOpts++ = '\0';

    // Parse the command line options

    while (( opt = getopt_long( argc, argv, shortOptionString, gOption, NULL )) != -1 )
    {
        switch ( opt )
        {
           case 0: 
            {
                // getopt_long returns 0 for entries where flag is non-NULL

                break;
            }

            case OPT_BINARY:
            {
                gBinary = 1;
                break;
            }

            case OPT_DEBUG:
            {
                gDebug = 1;
                break;
            }

            case OPT_EXECUTE:
            {
                gExecuteStr = optarg;
                break;
            }

            case OPT_MONITOR:
            {
                gMonitor = 1;
                break;
            }

            case OPT_SELECT:
            {
                gSelect = 1;
                break;
            }

            case OPT_VERBOSE:
            {
                gVerbose = 1;
                break;
            }

            case OPT_VERSION:
            {   
                printf( "gpio-event: SVN Revision: %d\n", SVN_REVISION );
                exit( 0 );
            }

            case '?':
            default:
            {
                fprintf( stderr, "opt:%d\n", opt );
            }
            case OPT_HELP:
            {
                Usage();
                exit( 1 );
            }
        }
    }
    argc -= optind;
    argv += optind;
    
    /* Original gpio-event */
    /*
    if (( fs = fopen( "/dev/gpio-geiger", "r" )) == NULL )
    {
        perror( "Check to make sure gpio_event_drv has been loaded. Unable to open /dev/gpio-event" );
        exit( 1 );
    }
    */

    if (( fs = fopen( "/dev/gpio-geiger", "r" )) == NULL )
    {
        perror( "Check to make sure gpio_event_drv has been loaded. Unable to open /dev/gpio-event" );
        exit( 1 );
    }
    
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

    int *inputId = malloc(sizeof(int));

	// Setup Read Geiger Thread
	pthread_t geiger_handler;
	pthread_create(&geiger_handler, NULL, geiger_handler_function, inputId); 
	fcntl( fileno(fs), F_SETOWN, geiger_handler);
	fcntl( fileno(fs), F_SETFL, FASYNC);
    // End Read Geiger Thread Setup
    printf("Set FASYNC flag and created geiger_handler thread\n");

    /* Adding only FASYNC to the original gpio-event app code*/

    // set this process as owner of the device file
    //printf("Setting file owner.\n");
    //int fd = fileno(fs);
    //fcntl( fd, F_SETOWN, getpid());
    
    // set FASYNC flag on the device file to enable SIGIO notifications
    //printf("Setting FASYNC flag.\n");
    //int oflags = fcntl(fd, F_GETFL);
    //fcntl(fd, F_SETFL, oflags | FASYNC);
    //printf("Setting FASYNC flag.. Done.\n");

    /* Adding FASYNC and opening mechanism to gpio-event from gpio-notify */
    /*
    // set FASYNC flag on the device file to enable SIGIO notifications
    printf("Setting FASYNC flag.\n");
    int oflags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, oflags | FASYNC);

    // open the gpio-event device
    int fs = open(GPIO_DEVICE_FILENAME, O_RDONLY | O_NONBLOCK);
    if(fs == -1) {
        fprintf(stderr, "Could not open %s\n", GPIO_DEVICE_FILENAME);
        
        exit(1);
    }
    
    printf("Opened device file %s on fd %d.\n", GPIO_DEVICE_FILENAME, fs);
    
    // set this process as owner of the device file
    printf("Setting file owner.\n");
    fcntl(fs, F_SETOWN, getpid());
    
    // set FASYNC flag on the device file to enable SIGIO notifications
    printf("Setting FASYNC flag.\n");
    int oflags = fcntl(fs, F_GETFL);
    fcntl(fs, F_SETFL, oflags | FASYNC);
    */
    
    // #fasync
    if ( gBinary )
    {
        ioctl( fileno( fs ), GPIO_EVENT_IOCTL_SET_READ_MODE, gBinary );
    }

    // Parse the gpio pins. Each pin can be followed by a modifier to indicate
    // the type of edges to monitor.

    for ( arg = 0; arg < argc; arg++ ) 
    {
        long                    gpio;
        char                   *gpioStr = argv[ arg ];
        char                   *endPtr;
        GPIO_EventMonitor_t     monitor;  // defined in gpio_event_driver.h 

        /* GPIO_EventMonitor_t struct declaration
        typedef struct
        {
            uint8_t                 gpio;               // gpio to monitor
            uint8_t                 onOff;              // 0 = stop monitoring, 1 = start monitoring
            GPIO_EventEdgeType_t    edgeType;           // Monitor rising/falling/both edges?
            uint8_t                 debounceMilliSec;   // debounce time in milliseconds

        } GPIO_EventMonitor_t;
        */

        // Turn pin on or off
        //strtol: convert a string to a long integer
        gpio = strtol( gpioStr, &endPtr, 0 );
        if ( gpio < 0 )
        {
            monitor.gpio = -gpio;
            monitor.onOff = 0;
        }
        else
        {
            monitor.gpio = gpio;
            monitor.onOff = 1;
        }

        if ( monitor.gpio > 255 )
        {
            fprintf( stderr, "Expecting gpio to be in the range 1-255, found: %d\n", monitor.gpio );
            exit( 1 );
        }
        if ( !ispunct( *endPtr ))
        {
            fprintf( stderr, "Expecting punctuation character, found '%c'\n", *endPtr );
            exit( 1 );
        }
        endPtr++; //advances from the expected : to the next field

        printf("At edge-type switch.\n");
        switch ( *endPtr )
        {
            case 'r':
            case 'R':   monitor.edgeType = GPIO_EventRisingEdge;    break;

            case 'f':
            case 'F':   monitor.edgeType = GPIO_EventFallingEdge;   break;

            case 'b':
            case 'B':   monitor.edgeType = GPIO_EventBothEdges;     break;

            default:
            {
                fprintf( stderr, "Expecting r, f, or b, found '%c'\n", *endPtr );
                exit( 1 );
            }
        }

        // Setup debounce
        printf("At debounce.\n");
        switch ( *endPtr )
        monitor.debounceMilliSec = 0;
        while ( isalpha( *endPtr ))
        {
            endPtr++;
        }
        if ( *endPtr != '\0' )
        {
            if ( !ispunct( *endPtr ))
            {
                fprintf( stderr, "Expecting punctuation character, found '%c'\n", *endPtr );
                exit( 1 );
            }
            endPtr++;

            monitor.debounceMilliSec = strtol( endPtr, NULL, 0 );
        }

        /* Description of the ioctl call
         * int ioctl( ind d, int request, ...);
         * where d is an open file descriptor, request is a device depentd request code, 
         * ... is a pointer to data going/coming to/from the driver. 
         *
         * The second argument, GPIO_EVENT_IOCTRL_MONITOR_GPIO is a macro used to generate ioctl command numbers. 
         *
         * #define GPIO_EVENT_IOCTL_MONITOR_GPIO   _IOW( GPIO_EVENT_IOCTL_MAGIC, GPIO_EVENT_CMD_MONITOR_GPIO,  GPIO_EventMonitor_t ) // arg is GPIO_EventMonitor *
         *
         * where _IOW is defined in /usr/include/asm/ioctl.h and is used for an ioctl that writes data TO the driver. The driver can return sizeof(data_type) bytes to the user. 
         * _IOW(int type, int number, data_type) - similar to _IOR, but used to write data to the driver.
        */
        
        printf("At ioctl fileno check.\n");
        if ( ioctl( fileno( fs ), GPIO_EVENT_IOCTL_MONITOR_GPIO, &monitor ) != 0 )
        {
            perror( "ioctl GPIO_EVENT_IOCTL_MONITOR_GPIO failed" );
        }
        printf("At line after ioctl fileno check.\n");
    }

    if ( gMonitor || ( gExecuteStr != NULL ))
    {
        while ( 1 )
        {
            ssize_t numBytes;
            char    argStr[ 60 ];

            argStr[ 0 ] = '\0';

            if ( gSelect )
            {
                fd_set          readSet;
                struct  timeval tv;
                int     rc;

                printf( "Waiting for data " );
                fflush( stdout );

                while ( 1 )
                {
                    FD_ZERO( &readSet );
                    FD_SET( fileno( fs ), &readSet );
    
                    tv.tv_sec = 1;
                    tv.tv_usec = 0;
    
                    rc = select( fileno( fs ) + 1, &readSet, NULL, NULL, &tv );
    
                    if ( rc == -1 )
                    {
                        perror( "select failed" );
                    }
                    else
                    if ( rc > 0 )
                    {
                        // Data is available

                        break;
                    }
                    else
                    {
                        printf( "." );
                        fflush( stdout );
                    }
                }
                printf( "\n" );
            }

            if ( gBinary )
            {
                GPIO_Event_t    gpioEvent;

                if (( numBytes = fread( &gpioEvent, 1, sizeof( gpioEvent ), fs )) == sizeof( gpioEvent ))
                {
                    snprintf( argStr, sizeof( argStr ), "%2d %c %ld.%06ld",
                              gpioEvent.gpio,
                              (( gpioEvent.edgeType == GPIO_EventRisingEdge ) ? 'R' : 'F' ),
                              gpioEvent.time.tv_sec,
                              gpioEvent.time.tv_usec );

                    if ( gMonitor )
                    {
                        printf( "ReadB: '%s'\n", argStr );
                    }
                }
                else
                {
                    if ( numBytes > 0 )
                    {
                        fprintf( stderr, "Read unexpected number of bytes: %d, expecting %d\n",
                                 numBytes, sizeof( gpioEvent ));
                    }
                }
            }
            else
            {
                if ( fgets( argStr, sizeof( argStr ), fs ) != NULL )
                {
                    numBytes = strlen( argStr );

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
            }

            if (( gExecuteStr != NULL ) && ( argStr[ 0 ] != '\0' ))
            {
                char    cmdStr[ 256 ];

                snprintf( cmdStr, sizeof( cmdStr ), "%s %s", gExecuteStr, argStr );

                if ( gVerbose )
                {
                    printf( "Executing '%s'\n", cmdStr );
                }
                system( cmdStr );
            }
        }
    }

    fclose( fs );

    exit( 0 );
    return 0;

} // main


