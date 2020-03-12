/*------------------------------------------------------------------
 *  Drone Header File
 *------------------------------------------------------------------
 */

#ifndef IN4073_H__
#define IN4073_H__

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml.h"
#include "app_util_platform.h"
#include <math.h>

#define MAX_SPEED 750
#define MIN_SPEED 200

#define RED            22
#define YELLOW        24
#define GREEN        28
#define BLUE        30
#define INT_PIN        5


// Timers
#define TIMER_PERIOD    50 //50ms=20Hz (MAX 23bit, 4.6h)

void timers_init(void);

uint32_t get_time_us(void);

bool check_timer_flag(void);

void clear_timer_flag(void);

// GPIO
void gpio_init(void);

// Queue
#define QUEUE_SIZE 256
typedef struct {
    uint8_t Data[QUEUE_SIZE];
    uint16_t first, last;
    uint16_t count;
} queue;

void init_queue(queue *q);

void enqueue(queue *q, char x);

char dequeue(queue *q);

// UART
#define RX_PIN_NUMBER  16
#define TX_PIN_NUMBER  14
queue rx_queue;
queue tx_queue;
uint32_t last_correct_checksum_time;

void uart_init(void);

void uart_put(uint8_t);

// TWI
#define TWI_SCL    4
#define TWI_SDA    2

void twi_init(void);

bool i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);

bool i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);

// MPU wrapper
int16_t phi, theta, psi;
int16_t sp, sq, sr;
int16_t sax, say, saz;
uint8_t sensor_fifo_count;

void imu_init(bool dmp,
              uint16_t interrupt_frequency); // if dmp is true, the interrupt frequency is 100Hz - otherwise 32Hz-8kHz
void get_dmp_data(void);

void get_raw_sensor_data(void);

bool check_sensor_int_flag(void);

void clear_sensor_int_flag(void);

// Barometer
int32_t pressure;
int32_t temperature;

void read_baro(void);

void baro_init(void);

// ADC
uint16_t bat_volt;

void adc_init(void);

void adc_request_sample(void);

// Flash
bool spi_flash_init(void);

bool flash_chip_erase(void);

bool flash_write_byte(uint32_t address, uint8_t data);

bool flash_write_bytes(uint32_t address, uint8_t *data, uint32_t count);

bool flash_read_byte(uint32_t address, uint8_t *buffer);

bool flash_read_bytes(uint32_t address, uint8_t *buffer, uint32_t count);

int count_flash;

// BLE
queue ble_rx_queue;
queue ble_tx_queue;
volatile bool radio_active;

void ble_init(void);

void ble_send(void);

// Communication
#define HEADER 0xFF
bool header_flag;
bool type_flag;
uint8_t msg_length;
uint8_t mode_set;
uint8_t toggle_mode;
uint8_t lift_set;
uint8_t pitch_set;
uint8_t yaw_set;
uint8_t roll_set;
uint8_t P;
uint8_t P1;
uint8_t P2;

void run_communication(void);

void process_header(void);

uint8_t process_type(void);

void process_message(uint8_t);

void receive_packet();

void send_packets(uint8_t);

uint8_t check_crc(uint8_t[], uint8_t);

uint8_t create_crc(uint8_t[], uint8_t);

// Logging
#define Max_Flash_address 0x01FFFF
uint16_t read_pata;
uint16_t write_pata;
uint16_t logged;

bool erase_log();

bool write_log();

bool read_log();

// State Machine ENUM
bool demo_done;
typedef enum control_mode {
    SAFE = 0, PANIC = 1, MANUAL = 2, CALIBRATION = 3, YAW_CONTROL = 4, FULL_CONTROL = 5, RAW = 6, HEIGHT = 7,
    LOGGING = 9
} mode;
mode current_mode;

// Calibration
int32_t sr0, sp0, sq0, sr_calib, sp_calib, sq_calib;
int32_t phi0, theta0, phi_calib, theta_calib;
int32_t pressure0, pressure_calib;
int16_t saz0, saz_calib;

uint8_t cali_counter;

// Control
int16_t motor[4], ae[4];
int32_t yaw_error, roll_error, pitch_error;
int32_t N_needed, L_needed, M_needed;
int32_t roll_gain, roll_rate_error; // debug
uint8_t yaw_ufb, roll_ufb, pitch_ufb;
int16_t roll_rate, pitch_rate, yaw_rate;
int16_t pitch_angle, roll_angle;
int32_t P_bar;
int16_t az;

void run_filters_and_control();

void run_yaw_control();

void update_motors();

void set_yaw(uint8_t);

void set_roll(uint8_t);

void set_pitch(uint8_t);

void check_min_motors_vals();

// Safety

int32_t bat_MAN;
uint16_t bat_MA;
uint8_t bat_count;
#define TIMEOUT_COM_US 200000
#define MIN_BATTERY_LVL 1050

bool check_connection_timeout();

#endif // IN4073_H__