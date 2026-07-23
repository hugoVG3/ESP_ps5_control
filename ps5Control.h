#ifndef PS5_CONTROL_H
#define PS5_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_l2cap_bt_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bluetooth PSM constants */
#define BT_PSM_HID_CONTROL 0x0011
#define BT_PSM_HID_INTERRUPT 0x0013

/* PS5 DualSense HID Report Constants */
#define PS5_HID_REPORT_ID 0x31
#define PS5_HID_REPORT_SIZE 78
#define PS5_ANALOG_STICK_MIN 0
#define PS5_ANALOG_STICK_MAX 255
#define PS5_ANALOG_STICK_CENTER 128
#define PS5_TRIGGER_MIN 0
#define PS5_TRIGGER_MAX 255

/* Button definitions */
typedef enum {
    PS5_BTN_SQUARE = (1 << 0),
    PS5_BTN_CROSS = (1 << 1),
    PS5_BTN_CIRCLE = (1 << 2),
    PS5_BTN_TRIANGLE = (1 << 3),
    PS5_BTN_L1 = (1 << 4),
    PS5_BTN_R1 = (1 << 5),
    PS5_BTN_L2 = (1 << 6),
    PS5_BTN_R2 = (1 << 7),
    PS5_BTN_SHARE = (1 << 8),
    PS5_BTN_OPTIONS = (1 << 9),
    PS5_BTN_L3 = (1 << 10),
    PS5_BTN_R3 = (1 << 11),
    PS5_BTN_PS = (1 << 12),
    PS5_BTN_TOUCHPAD = (1 << 13),
    PS5_BTN_MUTE = (1 << 14)
} ps5_button_t;

/* D-Pad directions */
typedef enum {
    PS5_DPAD_NEUTRAL = 0x08,
    PS5_DPAD_UP = 0x00,
    PS5_DPAD_UP_RIGHT = 0x01,
    PS5_DPAD_RIGHT = 0x02,
    PS5_DPAD_DOWN_RIGHT = 0x03,
    PS5_DPAD_DOWN = 0x04,
    PS5_DPAD_DOWN_LEFT = 0x05,
    PS5_DPAD_LEFT = 0x06,
    PS5_DPAD_UP_LEFT = 0x07
} ps5_dpad_t;

/* Parsed PS5 controller input data */
typedef struct {
    uint16_t buttons;           /* Bitmask of all button states */
    ps5_dpad_t dpad;            /* D-pad state */
    uint8_t left_stick_x;       /* Left stick X axis (0-255) */
    uint8_t left_stick_y;       /* Left stick Y axis (0-255) */
    uint8_t right_stick_x;      /* Right stick X axis (0-255) */
    uint8_t right_stick_y;      /* Right stick Y axis (0-255) */
    uint8_t l2_trigger;         /* L2 trigger analog value (0-255) */
    uint8_t r2_trigger;         /* R2 trigger analog value (0-255) */
    
    /* IMU data (optional, for motion control) */
    int16_t accel_x;            /* Accelerometer X */
    int16_t accel_y;            /* Accelerometer Y */
    int16_t accel_z;            /* Accelerometer Z */
    int16_t gyro_x;             /* Gyroscope X */
    int16_t gyro_y;             /* Gyroscope Y */
    int16_t gyro_z;             /* Gyroscope Z */
    
    uint8_t battery;            /* Battery level (0-100) */
    uint8_t timestamp;          /* Report timestamp */
} ps5_input_report_t;

/* HID command structure */
typedef struct {
    uint8_t *data;
    uint16_t length;
    uint8_t type;
} hid_cmd_t;

/* Function declarations - Initialization */
void ps5_init_bluetooth(void);
void ps5_deinit_bluetooth(void);

/* Function declarations - Connection management */
void ps5_l2cap_init_services(void);
void ps5_l2cap_deinit_services(void);
long ps5_l2cap_reconnect(void);

/* Function declarations - Data I/O */
void ps5_l2cap_send_hid(hid_cmd_t *cmd, uint8_t cid);
void ps5SetBluetoothMacAddress(const uint8_t *baseMac);

/* Function declarations - Input parsing */
bool ps5_parse_hid_report(const uint8_t *data, uint16_t len, ps5_input_report_t *report);

/* Function declarations - Callbacks (internal) */
void ps5_l2cap_connect_ind_cback(uint8_t *bd_addr, uint16_t l2cap_cid,
                                  uint16_t psm, uint8_t l2cap_id);
void ps5_l2cap_config_ind_cback(uint16_t l2cap_cid, tL2CAP_CFG_INFO *p_cfg);
void ps5_l2cap_config_cfm_cback(uint16_t l2cap_cid, tL2CAP_CFG_INFO *p_cfg);
void ps5_l2cap_disconnect_ind_cback(uint16_t l2cap_cid, bool ack_needed);
void ps5_l2cap_data_ind_cback(uint16_t l2cap_cid, BT_HDR *p_buf);

#ifdef __cplusplus
}
#endif

#endif /* PS5_CONTROL_H */
