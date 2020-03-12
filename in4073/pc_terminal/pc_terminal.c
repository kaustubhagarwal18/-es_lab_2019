#include "RS232_serial.h"
#include "term_serial.h"
#include <stdlib.h>
#include <math.h>
#include "commands.h"

#define JS_DEV    "/dev/input/js0"

#define HEADER 0xFF
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

/*----------------------------------------------------------------
 * CRC Functions
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
 * Embedded System Data Struct + Initialize Module
 *----------------------------------------------------------------
 */

typedef struct {
    uint16_t v_bat;                                              // Battery
    int32_t pressure;                                            // Pressure
    uint8_t drone_mode, toggle_mode;                             // ES Mode
    uint16_t set_lift, set_roll, set_pitch, set_yaw;             // ES Setpoints
    uint8_t P, P1, P2;                                           // Controllers
    int16_t yaw_rate, roll_rate, pitch_rate, phi, theta;         // Drone Velocities and Angles.
    uint16_t ae_0, ae_1, ae_2, ae_3;                             // Motor Values
} es_data;

es_data init_data() {
    es_data data;
    data.v_bat = 1100;
    data.pressure = 0;
    data.drone_mode = SAFE;
    data.toggle_mode = 0;
    data.set_lift = 0;
    data.set_roll = 127;
    data.set_pitch = 127;
    data.set_yaw = 127;
    data.P = 0;
    data.P1 = 0;
    data.P2 = 0;
    data.yaw_rate = 0;
    data.roll_rate = 0;
    data.pitch_rate = 0;
    data.phi = 0;
    data.theta = 0;
    data.ae_0 = 0;
    data.ae_1 = 0;
    data.ae_2 = 0;
    data.ae_3 = 0;
    return data;
}

/*----------------------------------------------------------------
 * Print Data
 *----------------------------------------------------------------
 */

/*----------------------------------------------------------------
 * Flag Check Modules
 *----------------------------------------------------------------
 */

int check_allow_mode_switch(commands js_commands,int es_mode) {
    int allow_mode_switch = 1;
    if (es_mode!=PANIC && js_commands.mode != PANIC) {
        if (js_commands.roll != 127 || js_commands.pitch != 127 || js_commands.yaw != 127 || js_commands.lift != 0)
            allow_mode_switch = 0;
        else
            allow_mode_switch = 1;
    }
    return allow_mode_switch;
}

/*----------------------------------------------------------------
 * Send Packet
 *----------------------------------------------------------------
 */
void send_packets(commands setpoints, int type) {
    uint8_t *packet_buffer;
    switch (type) {
        case 1:
            packet_buffer = (uint8_t *) calloc(5, sizeof(uint8_t));
            packet_buffer[0] = HEADER;
            packet_buffer[1] = 0x01;
            packet_buffer[2] = setpoints.mode;
            packet_buffer[3] = setpoints.toggle_mode;
            packet_buffer[4] = create_crc(packet_buffer, 4);
            for (int i = 0; i < 5; i++)
                rs232_putchar(packet_buffer[i]);

            free(packet_buffer);
            break;
        case 2:
            packet_buffer = (uint8_t *) calloc(6, sizeof(uint8_t));
            packet_buffer[0] = HEADER;
            packet_buffer[1] = 0x02;
            packet_buffer[2] = setpoints.P;
            packet_buffer[3] = setpoints.P1;
            packet_buffer[4] = setpoints.P2;
            packet_buffer[5] = create_crc(packet_buffer, 5);
            for (int i = 0; i < 6; i++)
                rs232_putchar(packet_buffer[i]);

            free(packet_buffer);
            break;
        case 3:
            packet_buffer = (uint8_t *) calloc(7, sizeof(uint8_t));
            packet_buffer[0] = HEADER;
            packet_buffer[1] = 0x03;
            packet_buffer[2] = setpoints.yaw;
            packet_buffer[3] = setpoints.pitch;
            packet_buffer[4] = setpoints.roll;
            packet_buffer[5] = setpoints.lift;
            packet_buffer[6] = create_crc(packet_buffer, 6);
            for (int i = 0; i < 7; i++)
                rs232_putchar(packet_buffer[i]);

            free(packet_buffer);
            break;
        default:
            break;
    }
}

/*----------------------------------------------------------------
 * Receiving Queue
 *----------------------------------------------------------------
 */

#define QUEUE_SIZE 256
typedef struct {
    uint8_t Data[QUEUE_SIZE];
    uint16_t first, last;
    uint16_t count;
} queue;

void init_queue(queue *q);

void enqueue(queue *q, char x);

char dequeue(queue *q);

int header_flag;
int type_flag;
uint8_t msg_length;

void init_queue(queue *q) {

    q->first = 0;
    q->last = QUEUE_SIZE - 1;
    q->count = 0;
}

void enqueue(queue *q, char x) {

    q->last = (q->last + 1) % QUEUE_SIZE;
    q->Data[q->last] = x;
    q->count += 1;
}

char dequeue(queue *q) {

    char x = q->Data[q->first];
    q->first = (q->first + 1) % QUEUE_SIZE;
    q->count -= 1;
    return x;
}

/*----------------------------------------------------------------
 * Receive Packet
 *----------------------------------------------------------------
 */

void process_header(queue *q) {
    uint8_t c;
    c = dequeue(q);
    if (c == HEADER) {
        header_flag = 1;
    }
}

uint8_t process_type(queue *q) {
    uint8_t msg_length;
    uint8_t c = dequeue(q);
    switch (c) {
        case 0x01:
            msg_length = 5;
            type_flag = 1;
            break;
        case 0x02:
            msg_length = 20;
            type_flag = 1;
            break;
        case 0x03:
            msg_length = 18;
            type_flag = 1;
            break;
        case 0x04:
            msg_length = 37;
            type_flag = 1;
            break;
        default:
            header_flag = 0;
            type_flag = 0;
            msg_length = 0;
    }
    return msg_length;
}

es_data process_message(uint8_t msg_length, queue *rx_queue, es_data data, FILE *f) {

    uint8_t *packet_buffer = (uint8_t *) calloc(msg_length, sizeof(uint8_t));

    packet_buffer[0] = HEADER;

    switch (msg_length) {

        case 5:
            packet_buffer[1] = 0x01;
            for (int i = 2; i < msg_length; i++)
                packet_buffer[i] = dequeue(rx_queue);

            if (!check_crc(packet_buffer, msg_length - 1)) {
                data.drone_mode = packet_buffer[2];
                data.toggle_mode = packet_buffer[3];

                printf("Mode: %3d ", data.drone_mode);
                if (data.toggle_mode)
                    printf(" RAW  ");
                else
                    printf(" DMP  ");
            }
            free(packet_buffer);
            header_flag = 0;
            type_flag = 0;
            break;

        case 20:
            packet_buffer[1] = 0x02;
            for (int i = 2; i < msg_length; i++)
                packet_buffer[i] = dequeue(rx_queue);

            if (!check_crc(packet_buffer, msg_length - 1)) {
                uint8_t sign[6];
                for (int i = 0; i < 6; i++)
                    sign[i] = (packet_buffer[2] >> i) & 0xFF;

                data.pressure = pow(-1, sign[5]) * ((((int16_t) packet_buffer[3]) << 24) + (packet_buffer[4] << 16) +
                                                    (packet_buffer[5] << 8) + (packet_buffer[6]));
                data.phi = pow(-1, sign[4]) * ((((int16_t) packet_buffer[7]) << 8) | packet_buffer[8]);
                data.theta = pow(-1, sign[3]) * ((((int16_t) packet_buffer[9]) << 8) | packet_buffer[10]);
                data.roll_rate = pow(-1, sign[2]) * ((((int16_t) packet_buffer[11]) << 8) | packet_buffer[12]);
                data.pitch_rate = pow(-1, sign[1]) * ((((int16_t) packet_buffer[13]) << 8) | packet_buffer[14]);
                data.yaw_rate = pow(-1, sign[0]) * ((((int16_t) packet_buffer[15]) << 8) | packet_buffer[16]);


                data.v_bat = (((uint16_t) packet_buffer[17]) << 8) | packet_buffer[18];

                printf("Angles (R P): %7d %7d ", data.phi, data.theta);
                printf("Gyro (Y R P): %7d %7d %7d ", data.yaw_rate, data.roll_rate, data.pitch_rate);
                printf("Pressure: %7d ", data.pressure);
                printf("V_bat: %4d ", data.v_bat);
            }
            free(packet_buffer);
            header_flag = 0;
            type_flag = 0;
            break;

        case 18:
            packet_buffer[1] = 0x03;
            for (int i = 2; i < msg_length; i++)
                packet_buffer[i] = dequeue(rx_queue);

            if (!check_crc(packet_buffer, msg_length - 1)) {

                data.ae_0 = (packet_buffer[2] << 8) | packet_buffer[3];
                data.ae_1 = (packet_buffer[4] << 8) | packet_buffer[5];
                data.ae_2 = (packet_buffer[6] << 8) | packet_buffer[7];
                data.ae_3 = (packet_buffer[8] << 8) | packet_buffer[9];
                data.P = packet_buffer[10];
                data.P1 = packet_buffer[11];
                data.P2 = packet_buffer[12];
                data.set_yaw = packet_buffer[13];
                data.set_pitch = packet_buffer[14];
                data.set_roll = packet_buffer[15];
                data.set_lift = packet_buffer[16];

                printf("Motor values: %3d %3d %3d %3d ", data.ae_0, data.ae_1, data.ae_2, data.ae_3);
                printf("Gain values P: %3d P1: %3d P2: %3d ", data.P, data.P1, data.P2);
                printf("Setpoints: %3d %3d %3d %3d\n", data.set_yaw, data.set_pitch, data.set_roll, data.set_lift);
            }
            free(packet_buffer);
            header_flag = 0;
            type_flag = 0;
            break;

        case 37:
            packet_buffer[1] = 0x04;

            for (int i = 2; i < msg_length; i++)
                packet_buffer[i] = dequeue(rx_queue);

            if (!check_crc(packet_buffer, msg_length - 1)) {
                uint8_t sign[6];
                for (int i = 0; i < 6; i++)
                    sign[i] = (packet_buffer[11] >> i) & 0xFF;

                uint32_t time = (packet_buffer[3] << 24) + (packet_buffer[4] << 16) + (packet_buffer[5] << 8) +
                                (packet_buffer[6]);
                uint8_t yaw_set = packet_buffer[7];
                uint8_t pitch_set = packet_buffer[8];
                uint8_t roll_set = packet_buffer[9];
                uint8_t lift_set = packet_buffer[10];
                int16_t roll_rate = pow(-1, sign[2]) * ((((int16_t) packet_buffer[12]) << 8) + packet_buffer[13]);
                int16_t pitch_rate = pow(-1, sign[1]) * ((((int16_t) packet_buffer[14]) << 8) + packet_buffer[15]);
                int16_t yaw_rate = pow(-1, sign[0]) * ((((int16_t) packet_buffer[16]) << 8) + packet_buffer[17]);
                int16_t phi = pow(-1, sign[4]) * ((((int16_t) packet_buffer[18]) << 8) + packet_buffer[19]);
                int16_t theta = pow(-1, sign[3]) * ((((int16_t) packet_buffer[20]) << 8) + packet_buffer[21]);
                int32_t P_bar = pow(-1, sign[5]) * ((((int32_t) packet_buffer[22]) << 24) + (packet_buffer[23] << 16) +
                                                    (packet_buffer[24] << 8) + (packet_buffer[25]));
                uint16_t ae_0 = (((uint16_t) packet_buffer[26]) << 8) + packet_buffer[27];
                uint16_t ae_1 = (((uint16_t) packet_buffer[28]) << 8) + packet_buffer[29];
                uint16_t ae_2 = (((uint16_t) packet_buffer[30]) << 8) + packet_buffer[31];
                uint16_t ae_3 = (((uint16_t) packet_buffer[32]) << 8) + packet_buffer[33];
                uint16_t v_bat = (((uint16_t) packet_buffer[34]) << 8) | packet_buffer[35];

                fprintf(f, "%6d,%6d,%6d,%6d,%6d,%6d,%7d,%7d,%7d,%7d,%7d,%7d,%6d,%6d,%6d,%6d,%6d\n", packet_buffer[2],
                        time, yaw_set, pitch_set, roll_set, lift_set, roll_rate, pitch_rate, yaw_rate, phi, theta,
                        P_bar, ae_0, ae_1, ae_2, ae_3, v_bat);
            } else
                printf("\n CRC Error in logging!\n");

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
    return data;
}

/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */

int main(int argc, char **argv) {

    // Initialise Terminal
    term_puts("\nTerminal program - Embedded Real-Time Systems\n");
    term_initio();

    // Initialise RS232
    rs232_open();
    int c;
    unsigned char x;
    queue rx_queue;

    // Initialise Receive Queue
    init_queue(&rx_queue);

    // Open File for logging
    FILE *f = fopen("basic_log.txt", "w");
    fprintf(f, "Mode,Time,yawSet,pitchSet,rollSet,liftSet,rollRate,pitchRate,yawRate,rollAngle,pitchAngle,P_bar,"
               "M1,M2,M3,M4,Battery\n");

    // Initialise static setpoints
    commands static_setpoints = init_com();

    // Initialise Joystick
    int fd_js;
    commands js_commands = init_com(); // Initialize dynamic setpoints
    if ((fd_js = open(JS_DEV, O_RDONLY)) < 0) {
        perror("Joystick Test Error");
        exit(1);
    }
    fcntl(fd_js, F_SETFL, O_NONBLOCK); // non-blocking mode

    int flag_packet1 = 0;
    int flag_packet2 = 5;
    int flag_packet3 = 10;

    // Initialise sum of commands to be sent
    commands user_setpoints = init_com();

    // Initialise data received
    es_data data = init_data();

    while (!user_setpoints.exit_flag) {

        // Get Joystick Commands
        if (data.drone_mode!=PANIC)
            js_commands.mode = SAFE;
        js_commands = getJS(fd_js, js_commands, data.drone_mode);

        // Get Keyboard Commands
        c = term_getchar_nb();
        // Reset static commands when ES goes to SAFE after PANIC
        if (data.drone_mode!=SAFE && user_setpoints.mode == PANIC)
            static_setpoints = init_com();
        else if (c != -1)
            static_setpoints = static_trimming(c, static_setpoints, user_setpoints,
                                               check_allow_mode_switch(js_commands,data.drone_mode));

        // Sum Static and Dynamic Commands
        user_setpoints = sum_commands(static_setpoints, js_commands, data.drone_mode);

        // Send Packet
        if (data.drone_mode!=LOGGING) {
            if ((flag_packet1 = flag_packet1 % 15) == 0)
                send_packets(user_setpoints, 1);
            if ((flag_packet2 = flag_packet2 % 15) == 0)
                send_packets(user_setpoints, 2); // Send control gains
            if ((flag_packet3 = flag_packet3 % 15) == 0)
                send_packets(user_setpoints, 3); // Send commands setpoints
            flag_packet2++;
            flag_packet3++;
        }

        // Receive Bytes and Enqueue
        if ((c = rs232_getchar_nb()) != -1) {
            x = (unsigned char) c;
            enqueue(&rx_queue, x);
        }

        // Dequeue Packet Received
        if (rx_queue.count >= 2 && !header_flag)
            process_header(&rx_queue);

        if (rx_queue.count && header_flag && !type_flag)
            msg_length = process_type(&rx_queue);

        if (rx_queue.count && (rx_queue.count >= (msg_length - 2)) && header_flag && type_flag)
            data = process_message(msg_length, &rx_queue, data, f);

    }

    fclose(f);
    term_exitio();
    rs232_close();
    term_puts("\n EXIT :D \n");

    return 0;
}
