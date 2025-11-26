#include "ps5Control.h" // Includes all necessary code

void setup() {
  Serial.begin(115200);
  
  // 1. Start the library and attempt to connect to the controller's MAC address
  if (ps5.begin("AA:BB:CC:DD:EE:FF")) {
    Serial.println("PS5 Controller initialization successful.");
  } else {
    Serial.println("PS5 Controller initialization failed.");
  }
}

void loop() {
  // 2. Check the connection status
  if (ps5.isConnected()) {
    // 3. Access button and analog state directly
    if (ps5.Cross()) {
      Serial.println("Cross button pressed!");
      // Send a command to the controller
      ps5.setLed(0, 0, 255); // Blue
      ps5.setRumble(50, 0); // Small rumble
      ps5.sendToController();
    } else {
      ps5.setRumble(0, 0); // Stop rumble
      ps5.sendToController();
    }

    // 4. Access analog stick values
    int8_t x = ps5.LStickX();
    int8_t y = ps5.LStickY();
    if (abs(x) > 5 || abs(y) > 5) {
      Serial.printf("Left Stick X: %d, Y: %d\n", x, y);
    }
  } else {
    Serial.println("Waiting for PS5 Controller...");
  }
  delay(10);
}
