# include "commands.h"

#define MIN(a, b) (((a)<(b))?(a):(b))
#define MAX(a, b) (((a)>(b))?(a):(b))

/*----------------------------------------------------------------
 * Keybord and Joystick Commands Initialize Module
 *----------------------------------------------------------------
 */

commands init_com() { // Initialize the command struct to 0
    commands com;
    com.mode = SAFE;
    com.toggle_mode = 0;
    com.roll = 0;
    com.pitch = 0;
    com.yaw = 0;
    com.lift = 0;
    com.P = 0;
    com.P1 = 0;
    com.P2 = 0;
    com.exit_flag = 0;
    return com;
}

/*----------------------------------------------------------------
 * Joystick and Keyboard Commands Sum Module
 *----------------------------------------------------------------
 */
commands sum_commands(commands static_setpoints, commands js_commands, int es_mode) {
    commands setpoints = init_com();

    if (js_commands.mode != PANIC && static_setpoints.mode != PANIC && es_mode!=PANIC)
        setpoints.mode = static_setpoints.mode;
    else if (es_mode != SAFE)
        setpoints.mode = PANIC;

    setpoints.toggle_mode = static_setpoints.toggle_mode;

    if (static_setpoints.exit_flag || js_commands.exit_flag)
        setpoints.exit_flag = 1;

    setpoints.roll = MAX(MIN(static_setpoints.roll + js_commands.roll, 254), 0);
    setpoints.pitch = MAX(MIN(static_setpoints.pitch + js_commands.pitch, 254), 0);
    setpoints.yaw = MAX(MIN(static_setpoints.yaw + js_commands.yaw, 254), 0);
    setpoints.lift = MAX(MIN(static_setpoints.lift + js_commands.lift, 254), 0);
    setpoints.P = MAX(MIN(static_setpoints.P, 254), 0);
    setpoints.P1 = MAX(MIN(static_setpoints.P1, 254), 0);
    setpoints.P2 = MAX(MIN(static_setpoints.P2, 254), 0);

    return setpoints;
}


/*----------------------------------------------------------------
 * Joystick Commands Module
 *----------------------------------------------------------------
 */

commands getJS(int fd, commands jsData, int es_mode) {
    struct js_event js;
    int roll, pitch, yaw, lift;
    while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)) {
        switch (js.type & ~JS_EVENT_INIT) {
            case JS_EVENT_BUTTON:
                if (js.number == 0) {
                    if (js.value == PANIC)
                        jsData.mode = PANIC;
                    else if (es_mode==SAFE)
                        jsData.mode = SAFE;
                    break;
                }
                break;
            case JS_EVENT_AXIS:
                switch (js.number) {
                    case 0:
                        roll = round((1 - js.value / 32767.0) * 127);
                        jsData.roll = roll;
                        break;
                    case 1:
                        pitch = round((js.value / 32767.0 + 1) * 127);
                        jsData.pitch = pitch;
                        break;
                    case 2:
                        yaw = round((js.value / 32767.0 + 1) * 127);
                        jsData.yaw = yaw;
                        break;
                    case 3:
                        lift = round((1 - js.value / 32767.0) * 127);
                        jsData.lift = lift;

                        break;
                    default:
                        break;
                }
                break;
        }
    }
    if (errno != EAGAIN) {
        perror("Joystick Error");
        jsData.exit_flag = 1;
    }
    return jsData;
}

/*----------------------------------------------------------------
 * Keyboard Commands
 *----------------------------------------------------------------
 */

commands static_trimming(uint8_t c, commands keys, commands setpoints, int allow_mode_switch) {
    int d;
    switch (c) {
        case 'a': //lift
            if (setpoints.lift >= 0 && setpoints.lift < 254)
                keys.lift += 1;
            break;
        case 'z':
            if (setpoints.lift > 0 && setpoints.lift <= 254)
                keys.lift -= 1;
            break;
        case 'q': //yaw
            if (setpoints.yaw > 0 && setpoints.yaw <= 254)
                keys.yaw -= 1;
            break;
        case 'w':
            if (setpoints.yaw >= 0 && setpoints.yaw < 254)
                keys.yaw += 1;
            break;
        case 27:
            if ((d = term_getchar_nb()) == 91) {
                if ((d = term_getchar_nb()) != -1) {
                    switch (d) {
                        case 65: //up key
                            if (setpoints.pitch > 0 && setpoints.pitch <= 254)
                                keys.pitch -= 1;
                            break;
                        case 66: //down key
                            if (setpoints.pitch >= 0 && setpoints.pitch < 254)
                                keys.pitch += 1;
                            break;
                        case 67: //right key
                            if (setpoints.roll > 0 && setpoints.roll <= 254)
                                keys.roll -= 1;
                            break;
                        case 68: //left key
                            if (setpoints.roll >= 0 && setpoints.roll < 254)
                                keys.roll += 1;
                            break;
                    }
                }
            } else
                keys.exit_flag = 1; //exit flag for the main loop
            break;
        case 'u': // P Control
            if (setpoints.P >= 0 && setpoints.P < 254)
                keys.P += 5;
            break;
        case 'j':
            if (setpoints.P > 0 && setpoints.P <= 254)
                keys.P -= 5;
            break;
        case 'i': // P1 Control
            if (setpoints.P1 >= 0 && setpoints.P1 < 254)
                keys.P1 += 5;
            break;
        case 'k':
            if (setpoints.P1 > 0 && setpoints.P1 <= 254)
                keys.P1 -= 5;
            break;
        case 'o': // P2 Control
            if (setpoints.P2 >= 0 && setpoints.P2 < 254)
                keys.P2 += 5;
            break;
        case 'l':
            if (setpoints.P2 > 0 && setpoints.P2 <= 254)
                keys.P2 -= 5;
            break;
        case 48:
            if (allow_mode_switch) {
                keys.mode = SAFE;
                break;
            } else
                break;
        case 49:
            keys.mode = PANIC;
            break;
        case 50:
            if (allow_mode_switch) {
                keys.mode = MANUAL;
                break;
            } else
                break;
        case 51:
            if (allow_mode_switch) {
                keys.mode = CALIBRATE;
                break;
            } else
                break;
        case 52:
            if (allow_mode_switch) {
                keys.mode = YAW_CONTROL;
                break;
            } else
                break;
        case 53:
            if (allow_mode_switch) {
                keys.mode = FULL_CONTROL;
                break;
            } else
                break;
        case 54:
            if (allow_mode_switch) {
                if (keys.mode == RAW) // If old mode is RAW
                    keys.toggle_mode = 1 - keys.toggle_mode; // Inverts 0 to 1 and vice-versa
                keys.mode = RAW;
                break;
            } else
                break;
        case 55:
            if (allow_mode_switch) {
                keys.mode = HEIGHT;
                break;
            } else
                break;
        case 56:
            if (allow_mode_switch) {
                keys.mode = WIRELESS;
                break;
            } else
                break;
        case 57:
            if (allow_mode_switch) {
                keys.mode = LOGGING;
                break;
            } else
                break;
        default:
            break;
    }
    return keys;
}