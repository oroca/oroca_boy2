/*
 * uart.cpp
 *
 *  Created on: 2017. 2. 13.
 *      Author: baram
 */


#include "uart.h"
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


typedef struct
{
   bool     is_open;
   uint8_t  channel;
   uint32_t baud;
   uint8_t  received_data;
   uint8_t  stop_bit;
   int      serial_handle;
   char     port_name[256];
 } uart_t;


uart_t uart_tbl[UART_CH_MAX];



static int  uartGetCFlagBaud(int baudrate);



bool uartInit(void)
{
  uint32_t i;


  for(i=0; i<UART_CH_MAX; i++)
  {
    uart_tbl[i].stop_bit          = 1;
    uart_tbl[i].serial_handle     = -1;
    uart_tbl[i].is_open           = false;
  }


  return true;
}

uint32_t uartOpen(uint8_t channel, char *port_name, uint32_t baud)
{
  uint32_t err_code  = OK;
  uart_t *p_uart = &uart_tbl[channel];
  struct termios newtio;

  uint32_t option = 0;


  if (channel >= UART_CH_MAX)
  {
    return 1;
  }


  strcpy(p_uart->port_name, port_name);
  p_uart->baud = uartGetCFlagBaud(baud);



  p_uart->serial_handle = open(port_name, O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(p_uart->serial_handle < 0)
  {
    printf("uartOpen :  Error opening serial port!\n");
    return false;
  }

  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings
  
  newtio.c_cflag = CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag      = 0;
  newtio.c_lflag      = 0;
  newtio.c_cc[VTIME]  = 0;
  newtio.c_cc[VMIN]   = 0;
  cfsetispeed(&newtio, baud);
  cfsetospeed(&newtio, baud);
  
  // clean the buffer and activate the settings for the port
  tcflush(p_uart->serial_handle, TCIFLUSH);
  tcsetattr(p_uart->serial_handle, TCSANOW, &newtio);



  p_uart->is_open = true;

  return err_code;
}

uint32_t uartClose(uint8_t channel)
{
  uint32_t err_code  = OK;
  uart_t *p_uart = &uart_tbl[channel];


  if (channel >= UART_CH_MAX)
  {
    return 1;
  }

  if (p_uart->is_open == true)
  {
    close(p_uart->serial_handle);
    //printf("close : %d\n", p_uart->serial_handle);
    p_uart->is_open = false;
  }


  return err_code;
}


#if 1
bool uartIsEnable(uint8_t channel)
{
  if (channel >= UART_CH_MAX)
  {
    return false;
  }

  return uart_tbl[channel].is_open;
}

void uartWaitForEnable(uint8_t channel, uint32_t timeout)
{
  uint32_t t_time;


  t_time = millis();
  while(1)
  {
    if (uartIsEnable(channel) == true)
    {
      break;
    }
    if ((millis()-t_time) >= timeout)
    {
      break;
    }
  }
}

uint32_t uartAvailable(uint8_t channel)
{

  uart_t *p_uart = &uart_tbl[channel];
  int bytes_available;


  ioctl(p_uart->serial_handle, FIONREAD, &bytes_available);

  return bytes_available;
}

void uartPutch(uint8_t channel, uint8_t ch)
{
  uartWrite(channel, &ch, 1 );
}

uint8_t uartGetch(uint8_t channel)
{
  if (uartIsEnable(channel) == false ) return 0;


  while(1)
  {
    if( uartAvailable(channel) ) break;
  }


  return uartRead(channel);
}

int32_t uartWrite(uint8_t channel, uint8_t *p_data, uint32_t length)
{
  int32_t  ret = 0;
  uart_t *p_uart = &uart_tbl[channel];

  if (uartIsEnable(channel) == false ) return 0;


  ret = write(p_uart->serial_handle, p_data, length);


  if (ret != length)
  {
    printf("write fail %d %d\n", ret, length);
    ret = 0;
  }

  return ret;
}

uint8_t uartRead(uint8_t channel)
{
  uint8_t ret = 0;
  uart_t *p_uart = &uart_tbl[channel];
  uint8_t data[1];

  if (read(p_uart->serial_handle, data, 1) == 1)
  {
    ret = data[0];
  }

  return ret;
}

int32_t uartPrintf(uint8_t channel, const char *fmt, ...)
{
  int32_t ret = 0;
  va_list arg;
  va_start (arg, fmt);
  int32_t len;
  static char print_buffer[255];


  if (uartIsEnable(channel) == false ) return 0;


  len = vsnprintf(print_buffer, 255, fmt, arg);
  va_end (arg);

  ret = uartWrite(channel, (uint8_t *)print_buffer, len);

  return ret;
}

int32_t uartPrint(uint8_t channel, uint8_t *p_str)
{
  int32_t index = 0;

  if (uartIsEnable(channel) == false ) return 0;

  while(1)
  {
    uartPutch(channel, p_str[index]);

    if (p_str[index] == 0)
    {
      break;
    }

    index++;

    if (index > 255)
    {
      break;
    }
  }


  return index;
}


int uartGetCFlagBaud(int baudrate)
{
  switch(baudrate)
  {
    case 1200:
      return B1200;    
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      return -1;
  }
}

#endif


#endif