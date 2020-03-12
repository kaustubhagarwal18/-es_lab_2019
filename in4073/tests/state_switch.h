//
// Created by koen on 9-5-19.
//

#ifndef ES_LAB_2019_STATE_SWITCH_H
#define ES_LAB_2019_STATE_SWITCH_H



// simulate update mode received from joystick
void *update_mode();

typedef enum control_mode {
    SAFE, PANIC, MANUAL
} mode;

mode current_mode;


#endif //ES_LAB_2019_STATE_SWITCH_H
