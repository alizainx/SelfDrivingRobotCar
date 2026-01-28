# SelfDrivingCar

Advanced autonomous robot car built with **ESP32**, featuring:
- Line following (3 IR sensors)
- Obstacle avoidance using A* pathfinding
- Red light detection with TinyML (Edge Impulse) on ESP32-CAM
- GPS waypoint navigation (NEO-6M)
- Remote control & emergency stop via Blynk IoT app
- Hardware emergency stop button with buzzer alerts

**Author:** Ali Zain  
**Date:** January 2026  


## Features

- Autonomous line following
- Dynamic obstacle avoidance with grid-based A* algorithm
- AI vision: stops at detected red traffic lights
- GPS navigation to target coordinates
- Mobile app control (Blynk)
- Safety: hardware + software emergency stops

## Hardware Requirements

- ESP32 DevKit (with PSRAM for camera)
- L298N motor driver
- 2× DC motors + chassis
- HC-SR04 ultrasonic sensor
- 3× TCRT5000 IR line sensors
- NEO-6M GPS module
- ESP32-CAM module
- Active buzzer
- Push button (emergency stop)
- Power source (e.g. 18650 batteries + voltage regulation)

## Pin Mapping

| Component              | Pin(s)                  |
|------------------------|-------------------------|
| Line sensors L/C/R     | 14 / 15 / 27            |
| Ultrasonic Trig/Echo   | 12 / 13                 |
| Buzzer                 | 32                      |
| Emergency button       | 0 (INPUT_PULLUP)        |
| Motor control          | 2,4 (left) / 5,18 (right), PWM 19 |
| GPS UART               | 16 (RX), 17 (TX)        |
| ESP32-CAM              | Standard pins (see code)|

## Software Requirements

- Arduino IDE with ESP32 board support
- Libraries:
  - NewPing
  - TinyGPS++
  - BlynkSimpleEsp32
  - EloquentTinyML
  - (esp_camera.h is included with ESP32 core)

- TinyML model (Edge Impulse):
  - Train a 96×96 RGB image classifier (red light / no light)
  - Export as Arduino library and include in project

## Installation

1. Clone or download this repository
2. Open `SelfDrivingRobotCar.ino` in Arduino IDE
3. Install required libraries via Library Manager
4. Update the following in the code:
   ```cpp
   char blynk_auth[] = "YOUR_AUTH_TOKEN";
   char wifi_ssid[]  = "YOUR_WIFI_SSID";
   char wifi_pass[]  = "YOUR_WIFI_PASSWORD";
   // and replace: ml_model.begin(your_model_name_ei);