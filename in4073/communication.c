#include "in4073.h"

/*----------------------------------------------------------------
 * Check Communication
 *----------------------------------------------------------------
 */
static uint32_t last_received_command;

bool check_connection_timeout() {
    if (get_time_us() > last_received_command + TIMEOUT_COM_US)
        return 1;
    else
        return 0;
};

/*----------------------------------------------------------------
 * CRC Code
 *----------------------------------------------------------------
 */

uint8_t create_crc(uint8_t packet[], uint8_t packet_size) {
    const uint8_t CRC7_POLY = 0x91;
    uint8_t i, j, crc = 0;

    for (i = 0; i < packet_size; i++) {
        crc ^= packet[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1)
                crc ^= CRC7_POLY;
            crc >>= 1;
        }
    }
    return crc;
}

uint8_t check_crc(uint8_t message[], uint8_t length) {
    uint8_t i, j, crc = 0;
    const uint8_t CRC7_POLY = 0x91;

    for (i = 0; i <= length; i++) {
        crc ^= message[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1)
                crc ^= CRC7_POLY;
            crc >>= 1;
        }
    }
    return crc;
}


/*----------------------------------------------------------------
 * Process Header - Checks for header
 *----------------------------------------------------------------
 */

void process_header() {
    uint8_t c;
    c = dequeue(&rx_queue);
    if (c == HEADER) {
        header_flag = 1;
    }
}


/*----------------------------------------------------------------
 *  * Process Type - Checks for type
 *----------------------------------------------------------------
 */

uint8_t process_type() {
    uint8_t msg_length;
    uint8_t c = dequeue(&rx_queue);
    switch (c) {
        case 0x01:
            msg_length = 5;
            type_flag = 1;
            break;
        case 0x02:
            msg_length = 6;
            type_flag = 1;
            break;
        case 0x03:
            msg_length = 7;
            type_flag = 1;
            break;
        default:
            header_flag = 0;
            type_flag = 0;
            msg_length = 0;
    }
    return msg_length;
}

/*----------------------------------------------------------------
 *  * Process Message - IF CRC write to variables
 *----------------------------------------------------------------
 */

void process_message(uint8_t msg_length) {
    uint8_t *packet_buffer = (uint8_t *) calloc(msg_length, sizeof(uint8_t));
    packet_buffer[0] = HEADER;
    switch (msg_length) {
        case 5:
            packet_buffer[1] = 0x01;
            for (uint8_t i = 2; i < msg_length; i++)
                packet_buffer[i] = dequeue(&rx_queue);

            if (!check_crc(packet_buffer, msg_length-1)) {
                if (current_mode != PANIC) { // Ignore Commands
                    mode_set = packet_buffer[2];
                    toggle_mode = packet_buffer[3];
                }
            }
            free(packet_buffer);
            header_flag = 0;
            type_flag = 0;
            break;
        case 6:
            packet_buffer[1] = 0x02;
            for (uint8_t i = 2; i < msg_length; i++)
                packet_buffer[i] = dequeue(&rx_queue);

            if (!check_crc(packet_buffer, msg_length-1)) {
                if (current_mode != PANIC) { // Ignore Commands
                    if (current_mode != SAFE) { // Ignore Commands
                        P = packet_buffer[2];
                        P1 = packet_buffer[3];
                        P2 = packet_buffer[4];
                    }
                }
            }
            free(packet_buffer);
            header_flag = 0;
            type_flag = 0;
            break;

        case 7:
            packet_buffer[1] = 0x03;
            for (uint8_t i = 2; i < msg_length; i++)
                packet_buffer[i] = dequeue(&rx_queue);

            if (!check_crc(packet_buffer, msg_length-1)) {
                if (current_mode != PANIC) { // Ignore Commands
                    if (current_mode != SAFE) { // Ignore Commands
                        yaw_set = packet_buffer[2];
                        pitch_set = packet_buffer[3];
                        roll_set = packet_buffer[4];
                        lift_set = packet_buffer[5];
                    }
                }
            }
            free(packet_buffer);
            header_flag = 0;
            type_flag = 0;
            break;

        default:
            free(packet_buffer);
            header_flag = 0;
            type_flag = 0;
            break;
    }

}

/*------------------------------------------------------------------
 * Send Package module
 *------------------------------------------------------------------
 */
void send_packets(uint8_t type) {
    uint8_t packet_buffer[40];
    uint8_t sign = 0;
    switch (type) {
        case 1:
            packet_buffer[0] = HEADER;
            packet_buffer[1] = 0x01;
            packet_buffer[2] = current_mode;
            packet_buffer[3] = toggle_mode;
            packet_buffer[4] = create_crc(packet_buffer, 4);
            for (uint8_t i = 0; i < 5; i++)
                uart_put(packet_buffer[i]);
            break;

        case 2:
            // Create a byte that holds the signs of signals
            sign |= (P_bar<0) << 5;
            sign |= (-roll_angle<0)<< 4;
            sign |= (pitch_angle<0)<<3;
            sign |= (-roll_rate<0)<<2;
            sign |= (-pitch_rate<0)<<1;
            sign |= (-yaw_rate<0)<<0;

            packet_buffer[0] = HEADER;
            packet_buffer[1] = 0x02;
            packet_buffer[2] = sign;
            packet_buffer[3] = (abs(P_bar) >> 24) & 0xFF;
            packet_buffer[4] = (abs(P_bar) >> 16) & 0xFF;
            packet_buffer[5] = (abs(P_bar) >> 8) & 0xFF;
            packet_buffer[6] = abs(P_bar) & 0xFF;
            packet_buffer[7] = (abs(roll_angle) >> 8) & 0xFF;
            packet_buffer[8] = abs(roll_angle) & 0xFF;
            packet_buffer[9] = (abs(pitch_angle) >> 8) & 0xFF;
            packet_buffer[10] = abs(pitch_angle) & 0xFF;
            packet_buffer[11] = (abs(roll_rate) >> 8) & 0xFF;
            packet_buffer[12] = abs(roll_rate) & 0xFF;
            packet_buffer[13] = (abs(pitch_rate) >> 8) & 0xFF;
            packet_buffer[14] = abs(pitch_rate) & 0xFF;
            packet_buffer[15] = (abs(yaw_rate) >> 8) & 0xFF;
            packet_buffer[16] = abs(yaw_rate) & 0xFF;
//            packet_buffer[17] = (bat_volt >> 8) & 0xFF;
//            packet_buffer[18] = bat_volt & 0xFF;
            packet_buffer[17] = (bat_MA >> 8) & 0xFF;
            packet_buffer[18] = bat_MA & 0xFF;
            packet_buffer[19] = create_crc(packet_buffer, 19);
            for (uint8_t i = 0; i < 20; i++)
                uart_put(packet_buffer[i]);
            break;

        case 3:
            packet_buffer[0] = HEADER;
            packet_buffer[1] = 0x03;
            packet_buffer[2] = (uint8_t)((ae[0] >> 8) & 0xFF);
            packet_buffer[3] = ae[0] & 0xFF;
            packet_buffer[4] = (uint8_t)((ae[1] >> 8) & 0xFF);
            packet_buffer[5] = ae[1] & 0xFF;
            packet_buffer[6] = (uint8_t)((ae[2] >> 8) & 0xFF);
            packet_buffer[7] = ae[2] & 0xFF;
            packet_buffer[8] = (uint8_t)((ae[3] >> 8) & 0xFF);
            packet_buffer[9] = ae[3] & 0xFF;
            packet_buffer[10] = P;
            packet_buffer[11] = P1;
            packet_buffer[12] = P2;
            packet_buffer[13] = yaw_set;
            packet_buffer[14] = pitch_set;
            packet_buffer[15] = roll_set;
            packet_buffer[16] = lift_set;
            packet_buffer[17] = create_crc(packet_buffer, 17);
            for (uint8_t i = 0; i < 18; i++)
                uart_put(packet_buffer[i]);
            break;

        case 4:;
            uint32_t j = 0;
            while(logged && flash_read_bytes(j, packet_buffer, 37)){
                for (uint8_t i = 0; i < 37; i++){
                    uart_put(packet_buffer[i]);
                    nrf_delay_ms(5);
                }
                nrf_gpio_pin_toggle(GREEN);
                j += 37;
                logged--;
            }
            break;

        default:
            break;
    }
}

/*------------------------------------------------------------------
 * Receive Package Module
 *------------------------------------------------------------------
 */

void receive_packet() {
    if (rx_queue.count >= 2 && !header_flag){
        nrf_gpio_pin_toggle(YELLOW);
        process_header();
        last_received_command = get_time_us(); // cable disconnect check
    }

    if (rx_queue.count && header_flag && !type_flag)
        msg_length = process_type();

    if (rx_queue.count && (rx_queue.count >= (msg_length - 2)) && header_flag && type_flag){
        process_message(msg_length);
        nrf_gpio_pin_toggle(YELLOW);
    }

}

/*------------------------------------------------------------------
 * Main run_communication
 *------------------------------------------------------------------
 */

void run_communication() {

    // Receive
    receive_packet();

    if (check_timer_flag()) {
        // Send Mode Packet.
        send_packets(1);
        // Send Sensor Packet.
        send_packets(2);
        // Send Controls Packet.
        send_packets(3);
        // Write Log.
        write_log();
        logged++;
        // Clear Flag.
        clear_timer_flag();
    }
}