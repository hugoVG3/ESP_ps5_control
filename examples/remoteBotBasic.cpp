/* Callback for controller input */
void handle_ps5_input(const ps5_input_report_t *report) {
    /* Drive motor control using left stick */
    int8_t fwd = ps5_get_left_stick_y();
    int8_t turn = ps5_get_right_stick_x();
    
    motor_set_speed(MOTOR_LEFT, fwd - turn);
    motor_set_speed(MOTOR_RIGHT, fwd + turn);
    
    /* Arm control with triggers */
    if (ps5_is_l2_pressed()) {
        arm_extend();
    }
    if (ps5_is_r2_pressed()) {
        arm_retract();
    }
    
    /* Button actions */
    if (ps5_button_pressed_once(PS5_BTN_CROSS)) {
        gripper_open();
    }
    if (ps5_button_pressed_once(PS5_BTN_CIRCLE)) {
        gripper_close();
    }
}

void handle_ps5_connect(bool connected) {
    if (connected) {
        ESP_LOGI("ROBOT", "PS5 controller connected - ready!");
        led_set_color(LED_GREEN);
    } else {
        ESP_LOGI("ROBOT", "PS5 controller disconnected");
        led_set_color(LED_RED);
        motor_stop_all();
    }
}

/* In main init: */
void app_main(void)
{
    ps5_init_bluetooth();
    ps5_l2cap_init_services();
    ps5_set_input_callback(handle_ps5_input);
    ps5_set_connect_callback(handle_ps5_connect);
    ps5_l2cap_reconnect();
    
    /* Main loop with non-blocking control */
    while (1) {
        if (ps5_is_button_pressed(PS5_BTN_PS)) {
            // Safe shutdown
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ps5_deinit_bluetooth();
}
