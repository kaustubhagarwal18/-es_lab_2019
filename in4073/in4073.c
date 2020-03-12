#include "in4073.h"

#define JS_SENSITIVITY 0 // 0 = highest joystick sensitivity; increase this number to lower sensitivity
#define UNSIGNED_ZERO_8B 127
#define OVERFLOW_CHECK

/*------------------------------------------------------------------
 * Yaw Functions
 *------------------------------------------------------------------
 */
void set_yaw(uint8_t js_received_yaw) {
    js_received_yaw = (uint16_t) js_received_yaw;

    // do nothing
    if (js_received_yaw == UNSIGNED_ZERO_8B) {
        // yaw clockwise if received value is > UNSIGNED_ZERO_8B
    } else if (js_received_yaw > UNSIGNED_ZERO_8B) {
        js_received_yaw = 2 * (js_received_yaw - 127) >> JS_SENSITIVITY;

        ae[0] += js_received_yaw;
        ae[2] += js_received_yaw;
#ifdef OVERFLOW_CHECK
        if (ae[1] < js_received_yaw)
            ae[1] = 0;
        else
            ae[1] -= js_received_yaw;
        if (ae[3] < js_received_yaw)
            ae[3] = 0;
        else
            ae[3] -= js_received_yaw;
#else
        ae[1] -= js_received_yaw;
        ae[3] -= js_received_yaw;
#endif
    } else {
        js_received_yaw = 2 * (127 - js_received_yaw) >> JS_SENSITIVITY;

        ae[1] += js_received_yaw;
        ae[3] += js_received_yaw;

#ifdef OVERFLOW_CHECK
        if (ae[0] < js_received_yaw)
            ae[0] = 0;
        else
            ae[0] -= js_received_yaw;

        if (ae[2] < js_received_yaw)
            ae[2] = 0;
        else
            ae[2] -= js_received_yaw;
#else
        ae[0] -= js_received_yaw;
        ae[2] -= js_received_yaw;
#endif
    }
}

/*------------------------------------------------------------------
 * Lift Functions
 *------------------------------------------------------------------
 */
void decrease_lift(uint16_t amount) {
    for (int i = 0; i < 4; i++) {
        if (ae[i] == 0) {
            continue;
        } else if (ae[i] < amount) {
            ae[i] = 0;
            continue;
        } else {
            ae[i] -= amount;
        }
    }
}

void set_lift(uint8_t js_received_lift) {
    for (int i = 0; i < 4; i++) {
        ae[i] = (uint16_t) js_received_lift;
        if (js_received_lift != 0) {
            ae[i] += MIN_SPEED;
        }
    }
}

/*------------------------------------------------------------------
 * Roll Functions
 *------------------------------------------------------------------
 */

void set_roll(uint8_t js_received_roll) {
    js_received_roll = (uint16_t)(js_received_roll);
    if (js_received_roll == UNSIGNED_ZERO_8B);
        // do nothing
    else if (js_received_roll > UNSIGNED_ZERO_8B) { // If roll is +
        js_received_roll = 2 * (js_received_roll - 127) >> JS_SENSITIVITY;
        // set motor 2 and 4
        ae[1] += js_received_roll;
#ifdef OVERFLOW_CHECK
        if (ae[3] < js_received_roll)
            ae[3] = 0;
        else
            ae[3] -= js_received_roll;
#else
        ae[3] -= js_received_roll;
#endif
    } else {
        js_received_roll = 2 * (127 - js_received_roll) >> JS_SENSITIVITY; // scale
#ifdef OVERFLOW_CHECK
        if (ae[1] < js_received_roll)
            ae[1] = 0;
        else
            ae[1] -= js_received_roll;
#else
        ae[1] -= js_received_roll;
#endif
        ae[3] += js_received_roll;

    }
}

/*------------------------------------------------------------------
 * Pitch Functions
 *------------------------------------------------------------------
 */

void set_pitch(uint8_t js_received_pitch) {
    js_received_pitch = (uint16_t) js_received_pitch;

    if (js_received_pitch == UNSIGNED_ZERO_8B);
        // do nothing
    else if (js_received_pitch > UNSIGNED_ZERO_8B) { // If pitch is +
        js_received_pitch = 2 * (js_received_pitch - 127) >> JS_SENSITIVITY;

        // set motor 1 and 3
        ae[0] += js_received_pitch;
#ifdef OVERFLOW_CHECK
        if (ae[2] < js_received_pitch)
            ae[2] = 0;
        else
            ae[2] -= js_received_pitch;
#else
        ae[2] -= js_received_pitch;
#endif
    } else {
        js_received_pitch = 2 * (127 - js_received_pitch) >> JS_SENSITIVITY;
#ifdef OVERFLOW_CHECK
        if (ae[0] < js_received_pitch)
            ae[0] = 0;
        else
            ae[0] -= js_received_pitch;
#else
        ae[0] -= js_received_pitch;
#endif
        ae[2] += js_received_pitch;
    }
}


/*------------------------------------------------------------------
 * Check Functions
 *------------------------------------------------------------------
 */
void check_min_motors_vals() {
    if (lift_set == 0) {
        for (int i = 0; i < 4; i++) {
            ae[i] = 0;
        }
    } else {
        for (int i = 0; i < 4; i++) {
            if (ae[i] == 0 || ae[i] < MIN_SPEED) {
                ae[i] = MIN_SPEED;
            }
        }
    }
}

int motors_on() {
    int motor_check = 0;
    for (int i = 0; i < 4; i++) {
        if (ae[i] != 0)
            motor_check++;
    }
    return motor_check;
}


/*------------------------------------------------------------------
 * Main Function - State Machine
 *------------------------------------------------------------------
 */

int main(void) {

    uart_init();
    gpio_init();
    timers_init();
    adc_init();
    twi_init();
    imu_init(true, 100);
    baro_init();
    spi_flash_init();
    ble_init();
    flash_chip_erase();
    uint8_t counter_safety = 0;
    demo_done = false;
    current_mode = SAFE;
    bat_volt = 1100;
    bat_count = 0;
    bat_MA = 0;
    bat_MAN = 0;

    while (!demo_done) {

        switch (current_mode) {
            case SAFE:
                run_communication();
                if (motors_on() == 0) {
                    current_mode = mode_set;
                    break;
                } else {
                    decrease_lift(1);
                    update_motors();
                }
                break;

            case PANIC:
                nrf_gpio_pin_toggle(RED);
                run_communication();
                current_mode = PANIC;
                decrease_lift(1);
                nrf_delay_ms(30);
                update_motors();
                if (motors_on() == 0)
                    current_mode = SAFE;
                nrf_gpio_pin_toggle(RED);

                break;

            case MANUAL:
                run_communication();
                current_mode = mode_set;
                set_lift(lift_set);
                set_pitch(pitch_set);
                set_yaw(yaw_set);
                set_roll(roll_set);
                check_min_motors_vals();
                update_motors();
                break;

            case CALIBRATION:
                run_communication();
                P_bar = pressure0 - pressure;
                switch (cali_counter) {
                    case 0 ... 199:
                        if (check_sensor_int_flag()) {
                            get_dmp_data();
                            read_baro();
                            sr_calib += sr;
                            sq_calib += sq;
                            sp_calib += sp;
                            phi_calib += phi;
                            theta_calib += theta;
                            pressure_calib += pressure;
                            saz_calib += saz;
                            cali_counter++;
                        }
                        break;
                    case 200:
                        sr0 = sr_calib / cali_counter;
                        sq0 = sq_calib / cali_counter;
                        sp0 = sp_calib / cali_counter;
                        phi0 = phi_calib / cali_counter;
                        theta0 = theta_calib / cali_counter;
                        pressure0 = pressure_calib / cali_counter;
                        saz0 = saz_calib / cali_counter;
                        cali_counter++;
                        nrf_gpio_pin_toggle(GREEN);
                        break;
                    case 201 :
                        if (mode_set != CALIBRATION) {
                            current_mode = mode_set;
                            cali_counter = 0;
                            sr_calib = sq_calib = sp_calib = 0;
                            theta_calib = phi_calib = 0;
                            pressure_calib = saz_calib = 0;
                            nrf_gpio_pin_toggle(GREEN);
                        }
                        break;
                    default:
                        break;
                }
                break;

            case YAW_CONTROL:
                run_communication();
                current_mode = mode_set;
                if (check_sensor_int_flag()) {
                    get_dmp_data();
                    read_baro();
                    P_bar = pressure0 - pressure;
                    set_lift(lift_set);
                    set_pitch(pitch_set);
                    set_roll(roll_set);
                    run_yaw_control();
                    check_min_motors_vals();
                    update_motors();
                }
                break;

            case FULL_CONTROL:
                run_communication();
                current_mode = mode_set;
                if (check_sensor_int_flag()) {
                    get_dmp_data();
                    read_baro();
                    P_bar = pressure0 - pressure;
                    set_lift(lift_set);
                    run_filters_and_control();
                    check_min_motors_vals();
                    update_motors();
                }
                break;

            case RAW:
                run_communication();
                current_mode = mode_set;
                if (toggle_mode); // RAW

                else; //DMP

                break;

            case HEIGHT:
                if (check_sensor_int_flag()) {
                    get_dmp_data();
                    read_baro();
                    P_bar = pressure0 - pressure;
                }
                break;

            case LOGGING:
                current_mode = LOGGING;
                send_packets(1); // One last mode signal
                send_packets(4); // Send all stored packets
                demo_done = 1;
                break;

            default:
                break;
        }

        // SAFETY
        if ((counter_safety = counter_safety % 50) == 0) {
            // Check Communication
            if (check_connection_timeout()) {
                current_mode = PANIC;
                if (!motors_on())
                    demo_done = 1;
            }
            // Check Battery
//            adc_request_sample();
//            if (bat_volt < MIN_BATTERY_LVL)
//                current_mode = PANIC;

            adc_request_sample();
            bat_MAN = bat_MAN + bat_volt - bat_MAN / 25;
            bat_MA = (uint16_t) bat_MAN / 25;

            if (bat_count < 25) {
                bat_count++;
            } else {
                if (bat_MA < MIN_BATTERY_LVL) {
                    current_mode = PANIC;
                }
            }
        }
    }

    nrf_delay_ms(100);
    NVIC_SystemReset();
}