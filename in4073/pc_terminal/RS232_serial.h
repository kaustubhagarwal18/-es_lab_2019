
#ifndef _RS232H_
#define _RS232H_

#include <termios.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>

int serial_device;
int fd_RS232;

void rs232_open(void);
void rs232_close(void);
int	rs232_getchar_nb();
int rs232_getchar();
int rs232_putchar(char c);

#endif