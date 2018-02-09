/*
 * util.c
 *
 *  Created on: 2017. 2. 13.
 *      Author: baram
 */


#include "util.h"

#if defined (__APPLE__)

 #include <stdio.h>
 #include <fcntl.h>
 #include <string.h>
 #include <unistd.h>
 #include <termios.h>
 #include <time.h>
 #include <sys/time.h>
 #include <sys/ioctl.h>

 #ifdef __MACH__
 #include <mach/clock.h>
 #include <mach/mach.h>
 #endif
 

uint32_t millis(void)
{
  double ret;

  struct timespec tv;
  #ifdef __MACH__ // OS X does not have clock_gettime, so here uses clock_get_time
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    tv.tv_sec = mts.tv_sec;
    tv.tv_nsec = mts.tv_nsec;
  #else
    clock_gettime(CLOCK_REALTIME, &tv);
  #endif
  
  ret = ((double)tv.tv_sec * 1000.0 + (double)tv.tv_nsec * 0.001 * 0.001);  

  return (uint32_t)ret;
}

#endif