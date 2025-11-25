# 🎮 ESP32 PS5 Controller Single-Header Library

> **This is a library meant to be used for communication between a PS5 remote (DualSense) and an ESP32 with wireless capability.**

This project condenses the Bluetooth communication stack required to interface with a Sony DualSense controller into a **single, lightweight header file**. This makes it incredibly easy to drop into any ESP32 Arduino project without managing complex library dependencies or multiple source files.

### ✨ Features

* **Zero-Dependency Setup:** Just one file (`ps5Controller.h`) to include in your project.
* **Plug & Play:** Connects to the DualSense controller via Classic Bluetooth.
* **Full Input Support:** Reads all digital buttons (Cross, Circle, D-Pad, Triggers, etc.).
* **Analog Precision:** Precise reading of Left/Right joysticks and L2/R2 analog triggers.
* **Haptic & Visual Feedback:** Control the programmable LED Lightbar (RGB) and Rumble motors.
* **Battery Status:** Monitor the controller's battery level and charging status.

---

### 📦 Hardware Requirements

* **ESP32 Development Board:** Must be a standard ESP32 (WROOM/WROVER) with **Classic Bluetooth** support.
    * *Note: This library relies on the ESP32 Bluetooth Classic stack. It generally does not work on ESP32-S3 or C3 variants as they typically support BLE only.*
* **Sony DualSense Controller:** The standard PS5 controller.

---

### 🛠️ Installation

Because this is a single-header library, you do not need to use the Arduino Library Manager.

1.  Download the **`ps5Controller.h`** file from this repository.
2.  Move the file directly into your Arduino sketch folder (e.g., `MyProject/ps5Controller.h`).
3.  In your main `.ino` file, add:
    ```cpp
    #include "ps5Controller.h"
    ```

---

### 🚀 Quick Start Guide

To use this library, you need the **Bluetooth MAC Address** of your PS5 controller.

#### 1. Finding your Controller's MAC Address
You can find the MAC address by connecting the controller to a PC or Android phone via Bluetooth and looking at the device properties, or by using a dedicated tool like "SixaxisPairTool" on Windows.

#### 2. The Code
Copy this into your main sketch file:

```cpp
#include "ps5Controller.h"

// 1. Enter the MAC address of your PS5 Controller here
// Format: "XX:XX:XX:XX:XX:XX"
const char* PS5_MAC = "1a:2b:3c:4d:5e:6f"; 

void setup() {
  Serial.begin(115200);
  
  Serial.print("Initializing Bluetooth...");
  
  // 2. Begin connection
  if (ps5.begin(PS5_MAC)) {
    Serial.println("Ready.");
  } else {
    Serial.println("Initialization failed!");
  }
}

void loop() {
  // 3. Check for connection
  if (ps5.isConnected()) {
    
    // Example: Print when Cross button is pressed
    if (ps5.Cross()) {
      Serial.println("Cross Button Pressed");
      
      // Feedback: Turn LED Red and Rumble
      ps5.setLed(255, 0, 0); 
      ps5.setRumble(100, 0);
      ps5.sendToController(); // Apply changes
    } else {
      // Reset Feedback
      ps5.setLed(0, 255, 0); // Green
      ps5.setRumble(0, 0);
      ps5.sendToController();
    }
    
    // Example: Read Analog Stick (Values -128 to 127)
    if (abs(ps5.LStickX()) > 10) {
      Serial.printf("Left Stick X: %d\n", ps5.LStickX());
    }

  } else {
    // Optional: Print waiting message every second
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      Serial.println("Waiting for controller...");
      lastPrint = millis();
    }
  }
}
````

-----

### 📚 API Documentation

The `ps5` object provides direct access to all controller features.

#### Buttons (Returns `bool`)

| Function | Description |
| :--- | :--- |
| `ps5.Right()`, `ps5.Left()`, `ps5.Up()`, `ps5.Down()` | D-Pad Directions |
| `ps5.Square()`, `ps5.Triangle()`, `ps5.Circle()`, `ps5.Cross()` | Face Buttons |
| `ps5.L1()`, `ps5.R1()` | Shoulder Bumpers |
| `ps5.L3()`, `ps5.R3()` | Stick Buttons (Clicking the stick in) |
| `ps5.Share()`, `ps5.Options()` | Create/Share and Options buttons |
| `ps5.PSButton()` | The central Playstation button |
| `ps5.Touchpad()` | Clicking the touchpad |

#### Analog Inputs

| Function | Range | Description |
| :--- | :--- | :--- |
| `ps5.LStickX()`, `ps5.LStickY()` | **-128 to 127** | Left Stick (Negative is Left/Up) |
| `ps5.RStickX()`, `ps5.RStickY()` | **-128 to 127** | Right Stick |
| `ps5.L2Value()`, `ps5.R2Value()` | **0 to 255** | Analog Triggers (0 = released) |

#### Outputs (LED & Rumble)

**Note:** After setting these values, you must call **`ps5.sendToController()`** for them to take effect.

| Function | Parameters | Description |
| :--- | :--- | :--- |
| `ps5.setLed(r, g, b)` | `0-255` for Red, Green, Blue | Changes the light bar color. |
| `ps5.setRumble(small, large)` | `0-255` Intensity | Sets motor vibration. `small` is high-freq, `large` is heavy. |
| `ps5.setFlashRate(on, off)` | `0-255` | Sets LED flashing intervals. |

#### System Status

| Function | Returns | Description |
| :--- | :--- | :--- |
| `ps5.Battery()` | `0-100` (approx) | Battery level. |
| `ps5.Charging()` | `bool` | True if connected to power. |

-----

### ❓ Troubleshooting

**1. The controller won't pair.**

  * Ensure the MAC address in `ps5.begin()` matches your controller exactly.
  * Put the controller in pairing mode (Hold **Share** + **PS Button** until it flashes rapidly) *before* booting the ESP32.
  * Try pressing the reset button on the back of the controller.

**2. Compilation Errors?**

  * Ensure you have the latest **ESP32 Board Manager** installed in Arduino IDE.
  * Select "ESP32 Dev Module" as your board.

-----

### 📄 License

This library is a single-header adaptation of open-source PS5 logic.
[MIT License](https://www.google.com/search?q=LICENSE)
