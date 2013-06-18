/****************************************************************************
*
*   Original gpio-event Copyright (c) 2006 Dave Hylands <dhylands@gmail.com>
*
*   gpio-notify 2008 by John Glassmyer <john.glassmyer@gmail.com>
*   -   added fasync notification method
*   -   removed event structures and file i/o
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
*   This driver allows multiple GPIO pins to be monitored and allows a user
*   mode program to be notified when the pin changes.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/version.h>

#include <asm/uaccess.h>
#include <asm/ioctls.h>

#include <asm/arch/hardware.h>
#include <asm/arch/pxa-regs.h>

#include "gpio-event-drv.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

#define GPIO_EVENT_DEV_NAME "gpio-event"

#define DEBUG_ENABLED   1

#if DEBUG_ENABLED
#   define DEBUG( flag, fmt, args... ) do { if ( gDebug ## flag ) printk( "%s: " fmt, __FUNCTION__ , ## args ); } while (0)
#else
#   define DEBUG( flag, fmt, args... )
#endif  

/* ---- Private Variables ------------------------------------------------ */

static char gBanner[] __initdata = KERN_INFO "GPIO Event Monitor 0.2 (gpio-notify version) Compiled: " __DATE__ " at " __TIME__ "\n";

static  int         gDebugTrace = 0;
static  int         gDebugIoctl = 0;
static  int         gDebugError = 1;
static  int         gLostEvents = 0;

static  struct ctl_table_header    *gSysCtlHeader;

static struct ctl_table gSysCtlSample[] =
{
    {
        .ctl_name       = 1,  
        .procname       = "lost-events",
        .data           = &gLostEvents,
        .maxlen         = sizeof( int ),
        .mode           = 0644,
        .proc_handler   = &proc_dointvec
    },
    {
        .ctl_name       = 101,
        .procname       = "debug-trace",
        .data           = &gDebugTrace,
        .maxlen         = sizeof( int ),
        .mode           = 0644,
        .proc_handler   = &proc_dointvec
    },
    {
        .ctl_name       = 102,
        .procname       = "debug-ioctl",
        .data           = &gDebugIoctl,
        .maxlen         = sizeof( int ),
        .mode           = 0644,
        .proc_handler   = &proc_dointvec
    },
    {
        .ctl_name       = 103,
        .procname       = "debug-error",
        .data           = &gDebugError,
        .maxlen         = sizeof( int ),
        .mode           = 0644,
        .proc_handler   = &proc_dointvec
    },
    { 0 }
};

static struct ctl_table gSysCtl[] =
{
    {
        .ctl_name       = CTL_GPIO_EVENT,
        .procname       = "gpio-event", 
        .mode           = 0555, 
        .child          = gSysCtlSample
    },
    { 0 }
};

/*
 * An instance of GPIO_FileData_t is maintained for file open
 */

#define GPIO_EVENT_BUFFER_SIZE  32

typedef struct
{
    struct list_head        list;
    wait_queue_head_t       waitQueue;

    volatile int            getIndex;
    volatile int            putIndex;
    volatile int            numEvents;

    GPIO_EventReadMode_t    readMode;

    char                    buffer[ GPIO_EVENT_BUFFER_SIZE ];
    int                     bufBytes;

    // jsg: queue of files/processes who have registered to receive asynchronous notifications
    struct fasync_struct    *async_queue;

} GPIO_FileData_t;

/*
 * An instance of GPIO_PinData_t is maintained for each GPIO line which is 
 * monitored, 
 */ 

typedef enum
{
    PIN_LOW             = 0,    // Matches level of GPIO line
    PIN_HIGH            = 1,
    PIN_BOUNCING_LOW,
    PIN_BOUNCING_HIGH,
} PinState_t;

typedef struct
{
    struct  list_head       list;               // list of all pins

    int                     gpio;               // The gpio line being monitored

    // We maintain two lists, a global list of pins, and a list associated with each open


    struct  timer_list      debounceTimer;      // Timer to wake u up after an edge
    uint8_t                 debounceMilliSec;   // debounce time in milliseconds
    char                    devName[ 16 ];      // gpio xx event

    GPIO_EventEdgeType_t    edgeType;   // Type of edge(s) we're looking for.

    PinState_t              pinState;          // Was the GPIO line low or high?
    
} GPIO_PinData_t;

static  volatile    int     gReportLostEvents = 1;

static  struct class       *gGpioEventClass = NULL;
static  struct  cdev        gGpioEventCDev;
static  dev_t               gGpioEventDevNum = 0;

static  spinlock_t          gFileListLock  = SPIN_LOCK_UNLOCKED;
static  spinlock_t          gPinListLock  = SPIN_LOCK_UNLOCKED;

static  LIST_HEAD( gFileList );
static  LIST_HEAD( gPinList );

static struct proc_dir_entry *gProcGpioEvent;
static struct proc_dir_entry *gProcPins;


/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

typedef struct
{
    unsigned    long    flags;
    struct list_head   *list;

} pin_seq_t;

/****************************************************************************
*
*  pin_seq_start
*
*   seq_file iterator which goes through the pins being monitored
*
****************************************************************************/

static void *pin_seq_start( struct seq_file *s, loff_t *pos )
{
    pin_seq_t  *ps;
    loff_t      i;

    s->private = NULL;

    if (( ps = kcalloc( 1, sizeof( pin_seq_t ), GFP_KERNEL )) == NULL )
    {
        return ERR_PTR( -ENOMEM );
    }
    s->private = ps;

    spin_lock_irqsave( &gPinListLock, ps->flags );

    if ( list_empty( &gPinList ))
    {
        DEBUG( Trace, "list_empty\n" );
        return NULL;
    }
    ps->list = gPinList.next;

    for ( i = 0; i < *pos; i++ ) 
    {
        if ( list_is_last( ps->list, &gPinList ))
        {
            DEBUG( Trace, "No item @ %llu\n", i + 1 );
            return NULL;
        }
        ps->list = ps->list->next;
    }


    DEBUG( Trace, "ps->list = 0x%08lx, *pos = %llu\n", (long)ps->list, *pos );

    return ps->list;

} // pin_seq_start

/****************************************************************************
*
*  pin_seq_show
*
*   seq_file iterator which goes through the pins being monitored
*
****************************************************************************/

static int pin_seq_show( struct seq_file *s, void *v )
{
    GPIO_PinData_t *pin = list_entry( v, GPIO_PinData_t, list );
    char           *edgeTypeStr;

    DEBUG( Trace, "v = 0x%08lx\n", (long)v );

    switch ( pin->edgeType )
    {
        case GPIO_EventRisingEdge:  edgeTypeStr = "Rising ";    break;
        case GPIO_EventFallingEdge: edgeTypeStr = "Falling";    break;
        case GPIO_EventBothEdges:   edgeTypeStr = "Both   ";    break;
        default:                    edgeTypeStr = "Unknown";    break;
    }

    seq_printf( s, "GPIO: %3d Edge: %s Debounce: %d msec\n", pin->gpio, edgeTypeStr, pin->debounceMilliSec );

    return 0;

} // pin_seq_show
	
/****************************************************************************
*
*  pin_seq_next
*
*   seq_file iterator which goes through the pins being monitored
*
****************************************************************************/

static void *pin_seq_next( struct seq_file *s, void *v, loff_t *pos )
{
    pin_seq_t  *ps = s->private;

    DEBUG( Trace, "v = 0x%08lx *pos = %llu\n", (long)v, *pos );

    if ( list_is_last( ps->list, &gPinList ))
    {
        DEBUG( Trace, "ps->list = 0x%08lx (end of list)\n", (long)ps->list );

        return NULL;
    }
    (*pos)++;
    ps->list = ps->list->next;

    DEBUG( Trace, "ps->list = 0x%08lx\n", (long)ps->list );

    return ps->list;

} // pin_seq_next

/****************************************************************************
*
*  pin_seq_stop
*
*   seq_file iterator which goes through the pins being monitored
*
****************************************************************************/

static void pin_seq_stop( struct seq_file *s, void *v )
{
    pin_seq_t  *ps = s->private;

    DEBUG( Trace, "v = 0x%08lx\n", (long)v );

    if ( ps != NULL )
    {
        spin_unlock_irqrestore( &gPinListLock, ps->flags );
        kfree( ps );
    }

} // pin_seq_stop

/****************************************************************************
*
*  pin_seq_ops
*
*   Ties all of the pin_seq_xxx routines together.
*
****************************************************************************/

static struct seq_operations pin_seq_ops = 
{
	.start = pin_seq_start,
	.next  = pin_seq_next,
	.stop  = pin_seq_stop,
	.show  = pin_seq_show
};

/****************************************************************************
*
*  pins_proc_open
*
*   Open method for /proc/gpio-event/pin
*
****************************************************************************/

static int pins_proc_open( struct inode *inode, struct file *file )
{
    DEBUG( Trace, "called\n" );

    return seq_open( file, &pin_seq_ops );
}

/****************************************************************************
*
*  pin_proc_ops
*
*   File operations for our /proc/gpio-event/pins file
*
****************************************************************************/

static struct file_operations pins_proc_ops = 
{
	.owner   = THIS_MODULE,
	.open    = pins_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};
	


/****************************************************************************
*
*  find_pin
*
*   Searches the list to see if 'gpio' is currently being monitored.
*
****************************************************************************/

static GPIO_PinData_t *find_pin( int gpio )
{
    struct  list_head   *pin;

    assert_spin_locked( &gPinListLock );

    list_for_each( pin, &gPinList )
    {
        GPIO_PinData_t *pinData = list_entry( pin, GPIO_PinData_t, list );

        if ( pinData->gpio == gpio )
        {
            return pinData;
        }
    }

    return NULL;

} // find_pin

/****************************************************************************
*
*  gpio_event_notify
*
*   Sends asynchronous notifications (SIGIO) to each process that has
*   registered itself when a pin change occurs.
*
****************************************************************************/

static void gpio_event_notify()
{
    unsigned long       flags;
    struct list_head   *file;

    spin_lock_irqsave( &gFileListLock, flags );

    list_for_each( file, &gFileList )
    {
        GPIO_FileData_t *fileData = list_entry( file, GPIO_FileData_t, list );

        // jsg: notify processes waiting on fasync notifications
        if(fileData->async_queue)
        {
            kill_fasync(&fileData->async_queue, SIGIO, POLL_IN);
        }
    }
    spin_unlock_irqrestore( &gFileListLock, flags );

} // gpio_event_notify

/****************************************************************************
*
*  gpio_event_irq
*
****************************************************************************/

#if ( LINUX_VERSION_CODE <= KERNEL_VERSION( 2, 6, 20 ))
#define REGS_PARAM      , struct pt_regs *regs
#define REGS_UNUSED     (void)regs
#else
#define REGS_PARAM      
#define REGS_UNUSED     
#endif

static irqreturn_t gpio_event_irq( int irq, void *dev_id REGS_PARAM )
{
    GPIO_PinData_t         *pinData = (GPIO_PinData_t *)dev_id;
    GPIO_Event_t            gpioEvent;
    int                     currLevel = (( GPLR( pinData->gpio ) & GPIO_bit( pinData->gpio )) != 0 );

    // We're called with interrupts disabled.

    (void)irq;
    REGS_UNUSED;

    gpioEvent.gpio = pinData->gpio;

    if ( pinData->debounceMilliSec == 0 )
    {
        // We assume that this is a clean signal

        pinData->pinState = (PinState_t)currLevel;

        if ( pinData->edgeType == GPIO_EventBothEdges )
        {
            // There's no register to tell which edge just occurred. So we
            // assume that it just changed into its current level.

            if ( currLevel )
            {
                // Pin is currently high, so this must be a rising edge

                gpioEvent.edgeType = GPIO_EventRisingEdge;
            }
            else
            {
                // Pin is currently low, so this must be a falling edge

                gpioEvent.edgeType = GPIO_EventFallingEdge;
            }
        }
        else
        {
            // If we're only monitoring one type of edge, then that's the one
            // that happened.

            gpioEvent.edgeType = pinData->edgeType;
        }
        
        // jsg: send asynchronous notifications
        gpio_event_notify();
    }
    else
    {
        gpioEvent.edgeType = 0;

        // If we need to debounce, then we need to monitor both edges, and
        // use the debounce timer to figure out the real state. So we don't
        // actually know which edge we just got. We use a state machine
        // to track things.

        switch ( pinData->pinState )
        {
            case PIN_LOW:
            {
                pinData->pinState = PIN_BOUNCING_HIGH;
                gpioEvent.edgeType = GPIO_EventRisingEdge;
                break;
            }

            case PIN_HIGH:
            {
                pinData->pinState = PIN_BOUNCING_LOW;
                gpioEvent.edgeType = GPIO_EventFallingEdge;
                break;
            }

            default:
            {
                break;
            }
        }

        if (( pinData->edgeType & gpioEvent.edgeType ) != 0 )
        {
            // This is an edge that the user is interested in - send it along.

            // jsg: send asynchronous notifications
            gpio_event_notify();
        }

        // Disable interrupts for our gpio to allow debounce to occur. The 
        // timer will re-enable the interrupt.

        disable_irq_nosync( irq );

        // Since we have no idea when in the current jiffy that the edge
        // occurred, we add 1 to the calculation to guarantee at least one
        // whole jiffy.

    }

    return IRQ_HANDLED;

} // gpio_event_irq

/****************************************************************************
*
*  gpio_event_timer
*
****************************************************************************/

void gpio_event_timer( unsigned long data )
{
    GPIO_PinData_t         *pinData = (GPIO_PinData_t *)data;

    // This function is called when the debounce timer for a gpio expires.
    // We record the state of the pin so that we can figure out what the 
    // next edge will be.

    pinData->pinState = (( GPLR( pinData->gpio ) & GPIO_bit( pinData->gpio )) != 0 );

    // Turn interrupts back on so we can catch the next edge

    enable_irq( IRQ_GPIO( pinData->gpio ));

} // gpio_event_timer

/****************************************************************************
*
*  gpio_event_monitor
*
****************************************************************************/

static int gpio_event_monitor( GPIO_EventMonitor_t *monitor )
{
    int             rc = 0;
    unsigned long   flags;
    GPIO_PinData_t *pinData;
    unsigned long   irqFlags;

    spin_lock_irqsave( &gPinListLock, flags );

    if ( monitor->onOff )
    {
        // Check to make sure we aren't already monitoring the gpio

        if (( pinData = find_pin( monitor->gpio )) != NULL )
        {
            // We are already monitoring the pin. Unmonitor the pin and then
            // proceed.

            monitor->onOff = 0;

            spin_unlock_irqrestore( &gPinListLock, flags );
            gpio_event_monitor( monitor );
            spin_lock_irqsave( &gPinListLock, flags );
        }

        if (( pinData = kcalloc( 1, sizeof( *pinData ), GFP_KERNEL )) == NULL )
        {
            DEBUG( Error, "GPIO %d: Out of memory\n", monitor->gpio );
            rc = -ENOMEM;
            goto out;
        }

        INIT_LIST_HEAD( &pinData->list );

        snprintf( pinData->devName, sizeof( pinData->devName ), "gpio %d event", monitor->gpio );

        // Note:
        //     Calling request_irq will automatically set the pin to be an input.

        irqFlags = 0;

        if ( monitor->debounceMilliSec == 0 )
        {
            // A clean signal is being presented, so we can just look for
            // a particular edge

            if (( monitor->edgeType | GPIO_EventRisingEdge ) != 0 )
            {
                irqFlags |= IRQF_TRIGGER_RISING;
            }
            if (( monitor->edgeType | GPIO_EventFallingEdge ) != 0 )
            {
                irqFlags |= IRQF_TRIGGER_FALLING;
            }
        }
        else
        {
            // Since we need to debounce, we need to look for both types of
            // edges, since we get both types of edges whenever a bounce
            // happens.

            irqFlags |= IRQF_TRIGGER_RISING;
            irqFlags |= IRQF_TRIGGER_FALLING;
        }

        if (( rc = request_irq( IRQ_GPIO( monitor->gpio ), gpio_event_irq, irqFlags, pinData->devName, pinData )) != 0 )
        {
            DEBUG( Error, "Unable to register irq for GPIO %d\n", monitor->gpio );
            kfree( pinData );
            goto out;
        }

        pinData->gpio             = monitor->gpio;
        pinData->edgeType         = monitor->edgeType;
        pinData->debounceMilliSec = monitor->debounceMilliSec;

        list_add_tail( &pinData->list, &gPinList );

        if (( GPLR( pinData->gpio ) & GPIO_bit( pinData->gpio )) == 0 )
        {
            pinData->pinState = PIN_LOW;
        }
        else
        {
            pinData->pinState = PIN_HIGH;
        }
    }
    else
    {
        if (( pinData = find_pin( monitor->gpio )) == NULL )
        {
            DEBUG( Error, "GPIO %d isn't being monitored\n", monitor->gpio );
            rc = -ENXIO;
            goto out;
        }

        // We've found the gpio being monitored - turn things off.

        free_irq( IRQ_GPIO( pinData->gpio ), pinData );

        list_del( &pinData->list );

        kfree( pinData );
    }

out:

    spin_unlock_irqrestore( &gPinListLock, flags );

    return rc;

} // gpio_event_monitor

/****************************************************************************
*
*   gpio_event_ioctl
*
*   Called to process ioctl requests
*
*****************************************************************************/

int gpio_event_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
    GPIO_FileData_t    *fileData;

    DEBUG( Trace, "type: '%c' cmd: 0x%x\n", _IOC_TYPE( cmd ), _IOC_NR( cmd ));

    fileData = file->private_data;

    switch ( cmd )
    {
        case GPIO_EVENT_IOCTL_MONITOR_GPIO:
        {
            GPIO_EventMonitor_t monitor;

            if ( copy_from_user( &monitor, (void *)arg, sizeof( monitor )) != 0 )
            {
                return -EFAULT;
            }
            return gpio_event_monitor( &monitor );
        }

        case GPIO_EVENT_IOCTL_SET_READ_MODE:
        {
            fileData->readMode = (GPIO_EventReadMode_t)arg;
            break;
        }

        case GPIO_EVENT_CMD_GENERATE_MODE:
        {
            gpio_event_notify();

			//GPIO_FileData_t *fileData;
		    // jsg: notify processes waiting on fasync notifications
	        //kill_fasync(&fileData->async_queue, SIGIO, POLL_IN);
			break;
            //return -ENOTTY;
        }


        case TCGETS:
        {
            // When cat opens this device, we get this ioctl
            return -ENOTTY;
        }

        default:
        {
            DEBUG( Error, "Unrecognized ioctl: '0x%x'\n", cmd );
            return -ENOTTY;
        }
    }

    return 0;

} // gpio_event_ioctl

/****************************************************************************
*
*  gpio_event_open
*
****************************************************************************/

static int gpio_event_open( struct inode *inode, struct file *file )
{
    unsigned long       flags;
    GPIO_FileData_t    *fileData;

    DEBUG( Trace, "gpio_event_open called, major = %d, minor = %d\n", MAJOR( inode->i_rdev ),  MINOR( inode->i_rdev ));

    // Allocate a per-open data structure

    if (( fileData = kcalloc( 1, sizeof( *fileData ), GFP_KERNEL )) == NULL )
    {
        return -ENOMEM;
    }

    INIT_LIST_HEAD( &fileData->list );

    init_waitqueue_head( &fileData->waitQueue );

    fileData->getIndex = 0;
    fileData->putIndex = 0;
    fileData->numEvents = 0;
    fileData->bufBytes = 0;

    fileData->readMode = GPIO_EventReadModeAscii;

    file->private_data = fileData;

    spin_lock_irqsave( &gFileListLock, flags );
    {
        list_add_tail( &fileData->list, &gFileList );
    }
    spin_unlock_irqrestore( &gFileListLock, flags );
    
    return 0;

} // gpio_event_open

/****************************************************************************
*
*  gpio_event_read
*
****************************************************************************/

static ssize_t gpio_event_read( struct file *file, char *buffer, size_t spaceRemaining, loff_t *ppos )
{
    int                 rc;
    ssize_t             bytesCopied = 0;
    ssize_t             bytesToCopy;
    GPIO_FileData_t    *fileData = file->private_data;

    // jsg: just return 0 bytes (i/o removed)
    
    return bytesCopied;
} // gpio_event_read

/****************************************************************************
*
*  gpio_event_poll - used by select & poll
*
****************************************************************************/

static unsigned int gpio_event_poll(struct file *file, poll_table *wait)
{
    unsigned long       flags;
    GPIO_FileData_t    *fileData = file->private_data;
    unsigned int        mask = 0;

    // jsg: just return no flags (no i/o ready)
    return mask;

} // gpio_event_poll

/*****************************************************************************
*
*  gpio_event_fasync
*
*   jsg: Add/remove an asynchronous reader to the SIGIO notification queue
*
****************************************************************************/

static int gpio_event_fasync(int fd, struct file *filp, int mode)
{
    GPIO_FileData_t *fileData = filp->private_data;
    
    return fasync_helper(fd, filp, mode, &fileData->async_queue);
}

/****************************************************************************
*
*  gpio_event_release
*
****************************************************************************/

static int gpio_event_release( struct inode *inode, struct file *file )
{
    unsigned long       flags;
    GPIO_FileData_t    *fileData = file->private_data;

    DEBUG( Trace, "gpio_event_release called\n" );

    spin_lock_irqsave( &gFileListLock, flags );
    {
        list_del( &fileData->list );
    }
    spin_unlock_irqrestore( &gFileListLock, flags );

    // jsg: remove this file from fasync notifcation queue
    gpio_event_fasync(-1, file, 0);

    kfree( fileData );

    return 0;

} // gpio_event_release

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
****************************************************************************/

struct file_operations gpio_event_fops =
{
    owner:      THIS_MODULE,
    ioctl:      gpio_event_ioctl,
    open:       gpio_event_open,
    poll:       gpio_event_poll,
    release:    gpio_event_release,
    read:       gpio_event_read,
    // jsg: add reference to fasync method
    fasync:     gpio_event_fasync
};

/****************************************************************************
*
*  gpio_event_init
*
*     Called to perform module initialization when the module is loaded
*
****************************************************************************/

static int __init gpio_event_init( void )
{
    int rc;

    DEBUG( Trace, "called\n" );

    printk( gBanner );

    // Get a major number

    if (( rc = alloc_chrdev_region( &gGpioEventDevNum, 0, 1, GPIO_EVENT_DEV_NAME )) < 0 )
    {
        printk( KERN_WARNING "sample: Unable to allocate major, err: %d\n", rc );
        return rc;
    }
    DEBUG( Trace, "allocated major:%d minor:%d\n", MAJOR( gGpioEventDevNum ), MINOR( gGpioEventDevNum ));

    // Register our proc entries.

    gProcGpioEvent = create_proc_entry( "gpio-event", S_IFDIR | S_IRUGO | S_IXUGO, NULL );
    if ( gProcGpioEvent == NULL )
    {
        return -ENOMEM;
    }
    gProcPins = create_proc_entry( "pins", 0444, gProcGpioEvent );
    if ( gProcPins != NULL )
    {
        gProcPins->proc_fops = &pins_proc_ops;
    }

#if ( LINUX_VERSION_CODE <= KERNEL_VERSION( 2, 6, 20 ))
    gSysCtlHeader = register_sysctl_table( gSysCtl, 0 );
    if ( gSysCtlHeader != NULL )
    {
        gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
    }
#else
    gSysCtlHeader = register_sysctl_table( gSysCtl );
#endif

    // Register our device. The device becomes "active" as soon as cdev_add 
    // is called.

    cdev_init( &gGpioEventCDev, &gpio_event_fops );
    gGpioEventCDev.owner = THIS_MODULE;

    if (( rc = cdev_add( &gGpioEventCDev, gGpioEventDevNum, 1 )) != 0 )
    {
        printk( KERN_WARNING "sample: cdev_add failed: %d\n", rc );
        return rc;
    }

    // Create a class, so that udev will make the /dev entry

    gGpioEventClass = class_create( THIS_MODULE, GPIO_EVENT_DEV_NAME );
    if ( IS_ERR( gGpioEventClass ))
    {
        printk( KERN_WARNING "sample: Unable to create class\n" );
        return -1;
    }

    class_device_create( gGpioEventClass, NULL, gGpioEventDevNum, NULL, GPIO_EVENT_DEV_NAME );

    return 0;

} // gpio_event_init

/****************************************************************************
*
*  gpio_event_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
****************************************************************************/

static void __exit gpio_event_exit( void )
{
    struct  list_head  *next;
    struct  list_head  *pin;
    GPIO_EventMonitor_t monitor;

    DEBUG( Trace, "called\n" );

    // If there are any pins which are currently being monitored, then we 
    // need to unmonitor them.

    memset( &monitor, 0, sizeof( monitor ));

    list_for_each_safe( pin, next, &gPinList )
    {
        GPIO_PinData_t *pinData = list_entry( pin, GPIO_PinData_t, list );

        monitor.gpio = pinData->gpio;

        gpio_event_monitor( &monitor );
    }

    // Deregister our driver

    class_device_destroy( gGpioEventClass, gGpioEventDevNum );
    class_destroy( gGpioEventClass );

    cdev_del( &gGpioEventCDev );

    if ( gSysCtlHeader != NULL )
    {
        unregister_sysctl_table( gSysCtlHeader );
    }
    remove_proc_entry( "pins", gProcGpioEvent );
    remove_proc_entry( "gpio-event", NULL );    
                                        
    unregister_chrdev_region( gGpioEventDevNum, 1 );

} // gpio_event_exit

/****************************************************************************/

module_init(gpio_event_init);
module_exit(gpio_event_exit);

MODULE_AUTHOR("John Glassmyer, Dave Hylands");
MODULE_DESCRIPTION("GPIO Event Driver (Notify-only)");
MODULE_LICENSE("Dual BSD/GPL");
