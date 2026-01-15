This project implements an advanced self-driving robot car using the ESP32 microcontroller. It integrates multiple features including line following, obstacle avoidance with A* pathfinding, AI-based vision for red light detection using TinyML, GPS navigation, remote control via Blynk app, and emergency stop safety mechanisms. The code is designed to be modular, efficient, and easy to extend, making it suitable for hobbyists, students, and makers.
The project is inspired by various open-source Arduino/ESP32 robot car implementations but enhanced with advanced algorithms and integrations for a complete autonomous experience. It's optimized for real-world deployment with simulated grid-based pathfinding and AI vision.
Author: Ali Zain
Date: January 15, 2026

**Features**

Line Following: Uses a 3-sensor IR array (TCRT5000) to follow black lines on a white surface.
Obstacle Avoidance: Detects obstacles via ultrasonic sensor (HC-SR04) and reroutes using A* algorithm on a simulated 10x10 grid.
AI Vision (Red Light Detection): Employs TinyML (via Edge Impulse) on ESP32-CAM to detect red lights and stop the robot.
GPS Navigation: Uses NEO-6M GPS module to navigate to a target location (e.g., coordinates in Islamabad).
Blynk App Control: Remote control via Blynk IoT app for speed adjustment and emergency stop.
Emergency Stop: Hardware button (GPIO 0) and app-based override with buzzer alerts.
Modular OOP Design: Robot actions managed in a SelfDrivingRobot class for clean code structure.
Safety First: Prioritizes emergency checks and alerts in the main loop.

**Hardware Requirements**

Microcontroller: ESP32 DevKit (or compatible board with ESP32-CAM for vision).
Motor Driver: L298N (dual H-bridge for DC motors).
Sensors:
Ultrasonic: HC-SR04 (for obstacle detection).
IR Array: 3x TCRT5000 (for line following).
GPS: NEO-6M (UART-connected).
Camera: ESP32-CAM (for AI vision).

**Actuators:**
Buzzer: Active buzzer for alerts.
Motors: 2x DC motors with wheels (chassis kit recommended).

Other:
Emergency Button: Push button on GPIO 0 (with pull-up).
Power: 5V/3.3V supply (battery pack for mobility).
Wi-Fi: For Blynk connectivity.


**Pinout Summary:**

Line Sensors: GPIO 14 (Left), 15 (Center), 27 (Right).
Ultrasonic: GPIO 12 (Trig), 13 (Echo).
Buzzer: GPIO 32.
Motors: GPIO 2/4 (Left), 5/18 (Right), 19 (PWM).
GPS: UART2 (GPIO 16 RX, 17 TX).
Emergency: GPIO 0 (INPUT_PULLUP).
ESP32-CAM: Standard pins (see code for config).

**Software Requirements**

IDE: Arduino IDE (with ESP32 board support installed via Boards Manager).
Libraries:
NewPing (for ultrasonic).
TinyGPS++ (for GPS parsing).
Blynk (BlynkSimpleEsp32 for app integration).
EloquentTinyML (for TinyML models).
esp_camera.h (included in ESP32 Arduino core).

TinyML Model: Train/export a model from Edge Impulse for red light detection (96x96 RGB images, 2 classes: red_light, no_light). Replace your_model_name_ei in code with your model's header/array.
Blynk Setup: Create a Blynk project with:
V0: Slider (0-255) for speed.
V1: Button (1=Stop) for emergency.
Replace blynk_auth, wifi_ssid, wifi_pass with your credentials.