#ifndef JS_H
#define JS_H

#include "joystick.h"
#include <math.h>
#include <stdint.h>
#include "term_serial.h"

#define SAFE 0x00
#define PANIC 0x01
#define MANUAL 0x02
#define CALIBRATE 0x03
#define YAW_CONTROL 0x04
#define FULL_CONTROL 0x05
#define RAW 0x06
#define HEIGHT 0x07
#define WIRELESS 0x08
#define LOGGING 0x09


typedef struct {
    int mode;
    int toggle_mode; // Only for RAW mode RAW-DMP switch
    int roll;
    int pitch;
    int yaw;
    int lift;
    int P;
    int P1;
    int P2;
    int exit_flag;
} commands;

commands init_com(void);
commands sum_commands(commands, commands, int);
commands getJS(int, commands,int);
commands static_trimming(uint8_t, commands, commands, int);


#endif