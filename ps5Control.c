#include "ps5Control.h"
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"

#define PS5_LOGI(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define PS5_LOGE(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)
#define PS5_LOGW(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
#define PS5_LOGD(format, ...) ESP_LOGD(TAG, format, ##__VA_ARGS__)

static const char *TAG = "PS5_CONTROL";

/* ============================================================================
   CONFIGURATION & CONSTANTS
   ============================================================================ */

#define PS5_ANALOG_STICK_DEADZONE     15    // Dead zone for sticks (0-255)
#define PS5_TRIGGER_DEADZONE          5     // Dead zone for triggers (0-255)
#define PS5_INVALID_ADDRESS_CHECK     {0, 0, 0, 0, 0, 0}
#define PS5_MIN_HID_REPORT_LENGTH     10
#define PS5_FULL_HID_REPORT_LENGTH    53

/* ============================================================================
   GLOBAL STATE VARIABLES
   ============================================================================ */

static uint8_t ps5_g_bd_addr[6] = {0};
static uint16_t ps5_l2cap_control_channel = 0;
static uint16_t ps5_l2cap_interrupt_channel = 0;
static bool ps5_connected = false;
static ps5_input_report_t ps5_last_report = {0};
static ps5_input_report_t ps5_prev_report = {0};

/* Callback function pointers */
static void (*ps5_input_callback)(const ps5_input_report_t *report) = NULL;
static void (*ps5_connect_callback)(bool connected) = NULL;

/* ============================================================================
   L2CAP CONFIGURATION
   ============================================================================ */

static tL2CAP_CFG_INFO ps5_cfg_info = {
    .mtu_present = true,
    .mtu = 672,
    .flush_tout_present = false,
    .flush_tout = 0xFFFF,
    .qos_present = false,
    .fcr_present = false,
    .fcs_present = false,
    .ext_flow_spec_present = false
};

static const tL2CAP_APPL_INFO ps5_dyn_info = {
    .pL2CA_ConnectInd_Cb = ps5_l2cap_connect_ind_cback,
    .pL2CA_ConnectCfm_Cb = NULL,
    .pL2CA_ConfigInd_Cb = ps5_l2cap_config_ind_cback,
    .pL2CA_ConfigCfm_Cb = ps5_l2cap_config_cfm_cback,
    .pL2CA_DisconnectInd_Cb = ps5_l2cap_disconnect_ind_cback,
    .pL2CA_Disconnect_Cfm_Cb = NULL,
    .pL2CA_DataInd_Cb = ps5_l2cap_data_ind_cback,
    .pL2CA_CongestionStatus_Cb = NULL,
    .pL2CA_TxComplete_Cb = NULL
};

/* ============================================================================
   ANALOG STICK PROCESSING WITH DEAD ZONE
   ============================================================================ */

static uint8_t ps5_apply_deadzone(uint8_t raw_value, uint8_t deadzone)
{
    int16_t centered = (int16_t)raw_value - 128;
    
    if (centered >= 0) {
        if (centered < deadzone) {
            return 128;
        }
        return 128 + (centered - deadzone);
    } else {
        if (centered > -deadzone) {
            return 128;
        }
        return 128 + (centered + deadzone);
    }
}

static int8_t ps5_normalize_stick(uint8_t raw_value, uint8_t deadzone)
{
    uint8_t processed = ps5_apply_deadzone(raw_value, deadzone);
    return (int8_t)(processed - 128);
}

/* ============================================================================
   HID REPORT PARSING WITH PROPER MASKING
   ============================================================================ */

bool ps5_parse_hid_report(const uint8_t *data, uint16_t len, ps5_input_report_t *report)
{
    if (!data || !report || len < PS5_MIN_HID_REPORT_LENGTH) {
        PS5_LOGE("Invalid HID report: null pointer or insufficient length (%d)", len);
        return false;
    }

    if (data[0] != PS5_HID_REPORT_ID) {
        PS5_LOGD("Unexpected HID report ID: 0x%02x (expected 0x%02x)", 
                 data[0], PS5_HID_REPORT_ID);
        return false;
    }

    /* -------- Analog Sticks (bytes 1-4) with Dead Zone -------- */
    report->left_stick_x = ps5_normalize_stick(data[1], PS5_ANALOG_STICK_DEADZONE);
    report->left_stick_y = ps5_normalize_stick(data[2], PS5_ANALOG_STICK_DEADZONE);
    report->right_stick_x = ps5_normalize_stick(data[3], PS5_ANALOG_STICK_DEADZONE);
    report->right_stick_y = ps5_normalize_stick(data[4], PS5_ANALOG_STICK_DEADZONE);

    /* -------- Triggers (bytes 5-6) with Dead Zone -------- */
    report->l2_trigger = ps5_apply_deadzone(data[5], PS5_TRIGGER_DEADZONE);
    report->r2_trigger = ps5_apply_deadzone(data[6], PS5_TRIGGER_DEADZONE);

    /* -------- Sequence Counter (byte 7) -------- */
    report->timestamp = data[7];

    /* -------- D-Pad and Buttons (byte 8) -------- */
    uint8_t byte8 = data[8];
    report->dpad = (ps5_dpad_t)(byte8 & 0x0F);
    report->buttons = 0;

    /* Mask and assign shape buttons */
    if (byte8 & 0x10) report->buttons |= PS5_BTN_SQUARE;
    if (byte8 & 0x20) report->buttons |= PS5_BTN_CROSS;
    if (byte8 & 0x40) report->buttons |= PS5_BTN_CIRCLE;
    if (byte8 & 0x80) report->buttons |= PS5_BTN_TRIANGLE;

    /* -------- Buttons (byte 9) -------- */
    uint8_t byte9 = data[9];
    if (byte9 & 0x01) report->buttons |= PS5_BTN_L1;
    if (byte9 & 0x02) report->buttons |= PS5_BTN_R1;
    if (byte9 & 0x04) report->buttons |= PS5_BTN_L2;
    if (byte9 & 0x08) report->buttons |= PS5_BTN_R2;
    if (byte9 & 0x10) report->buttons |= PS5_BTN_SHARE;
    if (byte9 & 0x20) report->buttons |= PS5_BTN_OPTIONS;
    if (byte9 & 0x40) report->buttons |= PS5_BTN_L3;
    if (byte9 & 0x80) report->buttons |= PS5_BTN_R3;

    /* -------- Buttons (byte 10) -------- */
    uint8_t byte10 = data[10];
    if (byte10 & 0x01) report->buttons |= PS5_BTN_PS;
    if (byte10 & 0x02) report->buttons |= PS5_BTN_TOUCHPAD;
    if (byte10 & 0x04) report->buttons |= PS5_BTN_MUTE;

    /* -------- IMU Data (Accelerometer & Gyroscope) -------- */
    if (len >= 25) {
        report->accel_x = (int16_t)((data[13] << 8) | data[14]);
        report->accel_y = (int16_t)((data[15] << 8) | data[16]);
        report->accel_z = (int16_t)((data[17] << 8) | data[18]);
        report->gyro_x = (int16_t)((data[19] << 8) | data[20]);
        report->gyro_y = (int16_t)((data[21] << 8) | data[22]);
        report->gyro_z = (int16_t)((data[23] << 8) | data[24]);
    }

    /* -------- Battery Level (byte 52) -------- */
    if (len >= PS5_FULL_HID_REPORT_LENGTH) {
        report->battery = data[52] & 0x7F;  /* Mask to 7 bits */
    }

    return true;
}

/* ============================================================================
   PUBLIC API - STATUS & CALLBACKS
   ============================================================================ */

void ps5_set_input_callback(void (*callback)(const ps5_input_report_t *report))
{
    ps5_input_callback = callback;
    PS5_LOGI("Input callback registered");
}

void ps5_set_connect_callback(void (*callback)(bool connected))
{
    ps5_connect_callback = callback;
    PS5_LOGI("Connection callback registered");
}

const ps5_input_report_t *ps5_get_last_report(void)
{
    return &ps5_last_report;
}

bool ps5_is_connected(void)
{
    return ps5_connected;
}

/* ============================================================================
   USER-FRIENDLY BUTTON API
   ============================================================================ */

bool ps5_is_button_pressed(ps5_button_t button)
{
    return (ps5_last_report.buttons & button) != 0;
}

bool ps5_button_pressed_once(ps5_button_t button)
{
    bool is_pressed_now = (ps5_last_report.buttons & button) != 0;
    bool was_pressed = (ps5_prev_report.buttons & button) != 0;
    return is_pressed_now && !was_pressed;
}

bool ps5_button_released(ps5_button_t button)
{
    bool is_pressed_now = (ps5_last_report.buttons & button) != 0;
    bool was_pressed = (ps5_prev_report.buttons & button) != 0;
    return !is_pressed_now && was_pressed;
}

/* ============================================================================
   USER-FRIENDLY ANALOG STICK API
   ============================================================================ */

int8_t ps5_get_left_stick_x(void)
{
    return ps5_last_report.left_stick_x;
}

int8_t ps5_get_left_stick_y(void)
{
    return ps5_last_report.left_stick_y;
}

int8_t ps5_get_right_stick_x(void)
{
    return ps5_last_report.right_stick_x;
}

int8_t ps5_get_right_stick_y(void)
{
    return ps5_last_report.right_stick_y;
}

bool ps5_is_left_stick_moved(void)
{
    return (ps5_last_report.left_stick_x != 0) || (ps5_last_report.left_stick_y != 0);
}

bool ps5_is_right_stick_moved(void)
{
    return (ps5_last_report.right_stick_x != 0) || (ps5_last_report.right_stick_y != 0);
}

/* ============================================================================
   USER-FRIENDLY TRIGGER API
   ============================================================================ */

uint8_t ps5_get_l2_trigger(void)
{
    return ps5_last_report.l2_trigger;
}

uint8_t ps5_get_r2_trigger(void)
{
    return ps5_last_report.r2_trigger;
}

bool ps5_is_l2_pressed(void)
{
    return ps5_last_report.l2_trigger > 128;
}

bool ps5_is_r2_pressed(void)
{
    return ps5_last_report.r2_trigger > 128;
}

float ps5_get_l2_trigger_normalized(void)
{
    return (float)ps5_last_report.l2_trigger / 255.0f;
}

float ps5_get_r2_trigger_normalized(void)
{
    return (float)ps5_last_report.r2_trigger / 255.0f;
}

/* ============================================================================
   USER-FRIENDLY D-PAD API
   ============================================================================ */

ps5_dpad_t ps5_get_dpad(void)
{
    return ps5_last_report.dpad;
}

bool ps5_is_dpad_up(void)
{
    return ps5_last_report.dpad == PS5_DPAD_UP || 
           ps5_last_report.dpad == PS5_DPAD_UP_LEFT || 
           ps5_last_report.dpad == PS5_DPAD_UP_RIGHT;
}

bool ps5_is_dpad_down(void)
{
    return ps5_last_report.dpad == PS5_DPAD_DOWN || 
           ps5_last_report.dpad == PS5_DPAD_DOWN_LEFT || 
           ps5_last_report.dpad == PS5_DPAD_DOWN_RIGHT;
}

bool ps5_is_dpad_left(void)
{
    return ps5_last_report.dpad == PS5_DPAD_LEFT || 
           ps5_last_report.dpad == PS5_DPAD_UP_LEFT || 
           ps5_last_report.dpad == PS5_DPAD_DOWN_LEFT;
}

bool ps5_is_dpad_right(void)
{
    return ps5_last_report.dpad == PS5_DPAD_RIGHT || 
           ps5_last_report.dpad == PS5_DPAD_UP_RIGHT || 
           ps5_last_report.dpad == PS5_DPAD_DOWN_RIGHT;
}

/* ============================================================================
   DEBUG & NAME LOOKUP
   ============================================================================ */

const char *ps5_get_button_name(ps5_button_t button)
{
    switch (button) {
        case PS5_BTN_SQUARE: return "SQUARE";
        case PS5_BTN_CROSS: return "CROSS";
        case PS5_BTN_CIRCLE: return "CIRCLE";
        case PS5_BTN_TRIANGLE: return "TRIANGLE";
        case PS5_BTN_L1: return "L1";
        case PS5_BTN_R1: return "R1";
        case PS5_BTN_L2: return "L2";
        case PS5_BTN_R2: return "R2";
        case PS5_BTN_SHARE: return "SHARE";
        case PS5_BTN_OPTIONS: return "OPTIONS";
        case PS5_BTN_L3: return "L3";
        case PS5_BTN_R3: return "R3";
        case PS5_BTN_PS: return "PS";
        case PS5_BTN_TOUCHPAD: return "TOUCHPAD";
        case PS5_BTN_MUTE: return "MUTE";
        default: return "UNKNOWN";
    }
}

const char *ps5_get_dpad_name(ps5_dpad_t dpad)
{
    switch (dpad) {
        case PS5_DPAD_UP: return "UP";
        case PS5_DPAD_UP_RIGHT: return "UP_RIGHT";
        case PS5_DPAD_RIGHT: return "RIGHT";
        case PS5_DPAD_DOWN_RIGHT: return "DOWN_RIGHT";
        case PS5_DPAD_DOWN: return "DOWN";
        case PS5_DPAD_DOWN_LEFT: return "DOWN_LEFT";
        case PS5_DPAD_LEFT: return "LEFT";
        case PS5_DPAD_UP_LEFT: return "UP_LEFT";
        case PS5_DPAD_NEUTRAL: return "NEUTRAL";
        default: return "UNKNOWN";
    }
}

void ps5_print_input_report(const ps5_input_report_t *report)
{
    if (!report) return;
    
    PS5_LOGI("=== PS5 Input Report ===");
    PS5_LOGI("Analog Sticks: LX=%d, LY=%d, RX=%d, RY=%d",
             report->left_stick_x, report->left_stick_y,
             report->right_stick_x, report->right_stick_y);
    PS5_LOGI("Triggers: L2=%d, R2=%d", report->l2_trigger, report->r2_trigger);
    PS5_LOGI("D-Pad: %s", ps5_get_dpad_name(report->dpad));
    PS5_LOGI("Buttons: 0x%04x", report->buttons);
    PS5_LOGI("IMU: AX=%d, AY=%d, AZ=%d, GX=%d, GY=%d, GZ=%d",
             report->accel_x, report->accel_y, report->accel_z,
             report->gyro_x, report->gyro_y, report->gyro_z);
    PS5_LOGI("Battery: %d%%", report->battery);
}

/* ============================================================================
   L2CAP SERVICE INITIALIZATION
   ============================================================================ */

static void ps5_l2cap_init_service(const char *name, uint16_t psm, uint8_t security_id)
{
    if (!L2CA_Register(psm, (tL2CAP_APPL_INFO *)&ps5_dyn_info)) {
        PS5_LOGE("Failed to register L2CAP PSM: 0x%04x", psm);
        return;
    }

    if (!BTM_SetSecurityLevel(false, name, security_id, 0, psm, 0, 0)) {
        PS5_LOGE("Failed to set security level for PSM: 0x%04x", psm);
        L2CA_Deregister(psm);
        return;
    }

    PS5_LOGI("L2CAP service initialized: %s (PSM: 0x%04x)", name, psm);
}

static void ps5_l2cap_deinit_service(const char *name, uint16_t psm)
{
    L2CA_Deregister(psm);
    PS5_LOGI("L2CAP service deinitialized: %s (PSM: 0x%04x)", name, psm);
}

void ps5_l2cap_init_services(void)
{
    ps5_l2cap_init_service("ps5-HIDC", BT_PSM_HID_CONTROL, BTM_SEC_SERVICE_FIRST_EMPTY);
    ps5_l2cap_init_service("ps5-HIDI", BT_PSM_HID_INTERRUPT, BTM_SEC_SERVICE_FIRST_EMPTY + 1);
}

void ps5_l2cap_deinit_services(void)
{
    ps5_l2cap_deinit_service("ps5-HIDC", BT_PSM_HID_CONTROL);
    ps5_l2cap_deinit_service("ps5-HIDI", BT_PSM_HID_INTERRUPT);
}

/* ============================================================================
   L2CAP CONNECTION MANAGEMENT
   ============================================================================ */

long ps5_l2cap_reconnect(void)
{
    uint8_t invalid_addr[] = PS5_INVALID_ADDRESS_CHECK;
    if (memcmp(ps5_g_bd_addr, invalid_addr, 6) == 0) {
        PS5_LOGE("PS5 device address not set");
        return -1;
    }

    long ret = L2CA_CONNECT_REQ(BT_PSM_HID_CONTROL, ps5_g_bd_addr, NULL, NULL);
    PS5_LOGI("L2CAP reconnect request sent, CID: %ld", ret);
    return ret;
}

void ps5_l2cap_send_hid(hid_cmd_t *cmd, uint8_t cid)
{
    if (!cmd || !cmd->data || cmd->length == 0) {
        PS5_LOGE("Invalid HID command");
        return;
    }

    uint16_t channel = (cid == 0) ? ps5_l2cap_control_channel : ps5_l2cap_interrupt_channel;
    if (channel == 0) {
        PS5_LOGE("L2CAP channel not connected");
        return;
    }

    BT_HDR *p_buf = (BT_HDR *)malloc(BT_DEFAULT_BUFFER_SIZE);
    if (!p_buf) {
        PS5_LOGE("Failed to allocate buffer for HID command");
        return;
    }

    p_buf->len = cmd->length;
    p_buf->offset = L2CAP_MIN_OFFSET;
    p_buf->layer_specific = 0;
    memcpy((uint8_t *)(p_buf + 1) + p_buf->offset, cmd->data, cmd->length);

    if (!L2CA_DataWrite(channel, p_buf)) {
        PS5_LOGE("Failed to send HID command");
        free(p_buf);
    } else {
        PS5_LOGD("HID command sent: %d bytes", cmd->length);
    }
}

void ps5SetBluetoothMacAddress(const uint8_t *baseMac)
{
    if (!baseMac) {
        PS5_LOGE("NULL MAC address pointer");
        return;
    }

    memcpy(ps5_g_bd_addr, baseMac, 6);
    esp_base_mac_addr_set(ps5_g_bd_addr);

    PS5_LOGI("Bluetooth MAC set: %02x:%02x:%02x:%02x:%02x:%02x",
             baseMac[0], baseMac[1], baseMac[2],
             baseMac[3], baseMac[4], baseMac[5]);
}

/* ============================================================================
   BLUETOOTH INITIALIZATION & DEINITIALIZATION
   ============================================================================ */

void ps5_init_bluetooth(void)
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;

    esp_err_t err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        PS5_LOGE("Bluetooth controller init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (err != ESP_OK) {
        PS5_LOGE("Bluetooth controller enable failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        PS5_LOGE("Bluedroid init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        PS5_LOGE("Bluedroid enable failed: %s", esp_err_to_name(err));
        return;
    }

    PS5_LOGI("Bluetooth initialized successfully");
}

void ps5_deinit_bluetooth(void)
{
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    PS5_LOGI("Bluetooth deinitialized");
}

/* ============================================================================
   L2CAP CALLBACK HANDLERS
   ============================================================================ */

static void ps5_l2cap_connect_ind_cback(uint16_t psm, uint16_t bd_addr_ptr,
                                        uint8_t id, uint8_t ertm)
{
    PS5_LOGI("L2CAP connect indication: PSM=0x%04x, ID=%u", psm, id);
    
    uint16_t lcid = L2CA_CONNECT_RSP((uint8_t *)bd_addr_ptr, id, psm,
                                     L2CAP_CONN_OK, L2CAP_CONN_OK,
                                     (tL2CAP_CFG_INFO *)&ps5_cfg_info);
    
    if (lcid > 0) {
        if (psm == BT_PSM_HID_CONTROL) {
            ps5_l2cap_control_channel = lcid;
            PS5_LOGI("HID Control channel established: 0x%04x", lcid);
        } else if (psm == BT_PSM_HID_INTERRUPT) {
            ps5_l2cap_interrupt_channel = lcid;
            PS5_LOGI("HID Interrupt channel established: 0x%04x", lcid);
            ps5_check_connection_status();
        }
    } else {
        PS5_LOGE("Failed to accept L2CAP connection");
    }
}

static void ps5_l2cap_config_ind_cback(uint16_t lcid, tL2CAP_CFG_INFO *p_cfg)
{
    PS5_LOGD("L2CAP config indication: LCID=0x%04x", lcid);
    
    if (p_cfg->result == L2CAP_CFG_OK) {
        L2CA_ConfigRsp(lcid, p_cfg);
    } else {
        PS5_LOGW("L2CAP configuration rejected");
        L2CA_DisconnectReq(lcid);
    }
}

static void ps5_l2cap_config_cfm_cback(uint16_t lcid, tL2CAP_CFG_INFO *p_cfg)
{
    PS5_LOGD("L2CAP config confirmation: LCID=0x%04x, result=%u", lcid, p_cfg->result);
    
    if (p_cfg->result != L2CAP_CFG_OK) {
        PS5_LOGW("L2CAP configuration failed");
        L2CA_DisconnectReq(lcid);
    }
}

static void ps5_l2cap_disconnect_ind_cback(uint16_t lcid, uint8_t ack_needed)
{
    PS5_LOGI("L2CAP disconnect indication: LCID=0x%04x", lcid);
    
    if (ack_needed) {
        L2CA_DisconnectRsp(lcid);
    }
    
    if (lcid == ps5_l2cap_control_channel) {
        ps5_l2cap_control_channel = 0;
    } else if (lcid == ps5_l2cap_interrupt_channel) {
        ps5_l2cap_interrupt_channel = 0;
    }
    
    bool was_connected = ps5_connected;
    ps5_connected = false;
    
    if (was_connected && ps5_connect_callback) {
        ps5_connect_callback(false);
    }
}

static void ps5_l2cap_data_ind_cback(uint16_t lcid, BT_HDR *p_buf)
{
    uint16_t len = p_buf->len;
    uint8_t *data = (uint8_t *)(p_buf + 1) + p_buf->offset;
    
    if (lcid == ps5_l2cap_interrupt_channel && len > 0) {
        PS5_LOGD("HID data received: %d bytes", len);
        
        /* Save previous report for button press detection */
        memcpy(&ps5_prev_report, &ps5_last_report, sizeof(ps5_input_report_t));
        
        if (ps5_parse_hid_report(data, len, &ps5_last_report)) {
            if (ps5_input_callback) {
                ps5_input_callback(&ps5_last_report);
            }
        }
    }
    
    osi_free(p_buf);
}

/* ============================================================================
   CONNECTION STATUS MANAGEMENT
   ============================================================================ */

static void ps5_check_connection_status(void)
{
    bool both_channels_connected = 
        (ps5_l2cap_control_channel > 0) && (ps5_l2cap_interrupt_channel > 0);
    
    if (both_channels_connected && !ps5_connected) {
        ps5_connected = true;
        PS5_LOGI("PS5 controller connected");
        if (ps5_connect_callback) {
            ps5_connect_callback(true);
        }
    }
}

void ps5_reset_state(void)
{
    ps5_l2cap_control_channel = 0;
    ps5_l2cap_interrupt_channel = 0;
    ps5_connected = false;
    memset(&ps5_last_report, 0, sizeof(ps5_last_report));
    memset(&ps5_prev_report, 0, sizeof(ps5_prev_report));
    PS5_LOGI("PS5 state reset");
}

