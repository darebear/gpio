#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

#include "gpio-event-drv.h"

#include "gpio.h"
#include "spi.h"

#define GPIO_DEVICE_FILENAME "/dev/gpio-event"

#define SPI_1_GPIO 72
#define SPI_2_GPIO 73
#define SPI_3_GPIO 68

#define NUMBER_OF_INPUTS 3

int inputGpio[] = {
    SPI_1_GPIO,
    SPI_2_GPIO,
    SPI_3_GPIO
};

pthread_mutex_t inputMutex[] = {
    PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER
};

pthread_cond_t inputCond[] = {
    PTHREAD_COND_INITIALIZER,
    PTHREAD_COND_INITIALIZER,
    PTHREAD_COND_INITIALIZER
};

int inputCount[NUMBER_OF_INPUTS];

int quit = 0;

struct timeval startTime;

// start or stop monitoring a gpio pin on the specified file
//   onOff = 0 to stop monitoring pin, 1 to start monitoring pin
void monitorPin(int gpio, int onOff, int fd) {
    GPIO_EventMonitor_t monitor;
    
    monitor.gpio = gpio;
    monitor.onOff = onOff;
    monitor.edgeType = GPIO_EventBothEdges;
    monitor.debounceMilliSec = 0;
    
    const char *startStop = onOff ? "Start" : "Stop";
    printf("%s monitoring pin %d on fd %d.\n", startStop, gpio, fd);
    
    // enable monitoring on the selected pins
    ioctl(fd, GPIO_EVENT_IOCTL_MONITOR_GPIO, &monitor);
}

int openGpioMonitor() {
    // open the gpio-event device
    int fd = open(GPIO_DEVICE_FILENAME, O_RDONLY | O_NONBLOCK);
    if(fd == -1) {
        fprintf(stderr, "Could not open %s\n", GPIO_DEVICE_FILENAME);
        
        exit(1);
    }
    
    printf("Opened device file %s on fd %d.\n", GPIO_DEVICE_FILENAME, fd);
    
    // set this process as owner of the device file
    printf("Setting file owner.\n");
    fcntl(fd, F_SETOWN, getpid());
    
    // set FASYNC flag on the device file to enable SIGIO notifications
    printf("Setting FASYNC flag.\n");
    int oflags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, oflags | FASYNC);
    
    return fd;
}

void *signalThreadProc(void *arg) {
    int inputStatus[NUMBER_OF_INPUTS];
    
    // specify which signals we will catch
    sigset_t waitSignals;
    sigemptyset(&waitSignals);
    sigaddset(&waitSignals, SIGIO);
    sigaddset(&waitSignals, SIGHUP);
    sigaddset(&waitSignals, SIGTERM);
    
    for(;;) {
        int signal;
        
        sigwait(&waitSignals, &signal);
        
        switch(signal) {
            case SIGIO: {
                // check whether each input pin has changed, and notify corresponding threads
                int i;
                for(i = 0; i < NUMBER_OF_INPUTS; i++) {
                    int currentStatus = gpio_status(inputGpio[i]);
                    
                    if(inputStatus[i] != currentStatus) {
                        
                        inputStatus[i] = currentStatus;
                        
                        // is it a rising edge?
                        if(currentStatus) {
                            pthread_mutex_lock(&inputMutex[i]);
                            
                            // signal the input thread to process the event
                            pthread_cond_signal(&inputCond[i]);
                            
                            pthread_mutex_unlock(&inputMutex[i]);
                        }
                        
                    }
                }
            }
            break;
            
            case SIGHUP: {
                int i;
                for(i = 0; i < NUMBER_OF_INPUTS; i++) {
                    inputCount[i] = 0;
                }
                
                gettimeofday(&startTime, NULL);
                
                printf("time: %d.%06d\n", (int)startTime.tv_sec, (int)startTime.tv_usec);
            }
            break;
            
            case SIGINT:
            case SIGTERM: {
                struct timeval endTime;
                gettimeofday(&endTime, NULL);
                
                printf("Caught SIGTERM. Exiting.\n");
                
                sleep(1);
                
                printf("time: %d.%06d\n", (int)endTime.tv_sec, (int)endTime.tv_usec);
                
                double secondsElapsed = 
                    (endTime.tv_sec - startTime.tv_sec)
                    +
                    (endTime.tv_usec - startTime.tv_usec) * 0.000001;
                
                printf("seconds elapsed: %.3f\n", secondsElapsed);
                
                int i;
                for(i = 0; i < NUMBER_OF_INPUTS; i++) {
                    printf("input count %d: %d (%f/sec)\n", i, inputCount[i], ((double)inputCount[i]) / secondsElapsed);
                }
                
                exit(0);
            }
            break;
        }
    }
}

void *inputThreadProc(void *arg) {
    int inputId = *(int *)arg;
    free(arg);
    
    for(;;) {
        pthread_mutex_lock(&inputMutex[inputId]);
        
        // wait until signalled by the signal handler thread
        pthread_cond_wait(&inputCond[inputId], &inputMutex[inputId]);
        
        inputCount[inputId]++;
        
        pthread_mutex_unlock(&inputMutex[inputId]);
    }
}

int main() {
    SPIinit(0);
    gpio_init();
    
    // ignore all signals in all threads
    sigset_t ignoreSignals;
    sigfillset(&ignoreSignals);
    pthread_sigmask(SIG_BLOCK, &ignoreSignals, NULL);
    
    int gpioFd = openGpioMonitor();
    
    pthread_t inputThread[NUMBER_OF_INPUTS];
    
    int i;
    for(i = 0; i < NUMBER_OF_INPUTS; i++) {
        // listen for this button's gpio
        monitorPin(inputGpio[i], 1, gpioFd);
        
        inputCount[i] = 0;
        
        int *inputId = malloc(sizeof(int));
        *inputId = i;
        
        // start the input listener thread
        pthread_create(&inputThread[i], NULL, inputThreadProc, inputId);
    }
    
    // print our process id
    printf("pid: %d\n", getpid());
    
    // print the start time
    gettimeofday(&startTime, NULL);
    printf("time: %d.%06d\n", (int)startTime.tv_sec, (int)startTime.tv_usec);
    
    // start the signal handler thread
    pthread_t signalThread;
    pthread_create(&signalThread, NULL, signalThreadProc, NULL);
    
    while(1) {
        sleep(30);
    }
    
    return 0;
}
