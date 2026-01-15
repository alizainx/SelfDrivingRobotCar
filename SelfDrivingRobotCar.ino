
// Complete Advanced Self-Driving Robot Car Code for ESP32

// Features: Line Following, Obstacle Avoidance with A* Pathfinding,
// AI Vision (Red Light Stop via TinyML), GPS Navigation,
// Blynk App Control, Emergency Stop Safety

// Author: Ali Zain

// Date: January 15, 2026

// Hardware: ESP32 DevKit, L298N Motor Driver, HC-SR04 Ultrasonic,
// NEO-6M GPS, TCRT5000 IR Array (3 sensors), ESP32-CAM for Vision,
// Buzzer, Emergency Button on GPIO 0

// Libraries: NewPing, TinyGPS++, BlynkSimpleEsp32, EloquentTinyML, esp_camera.h

// Note: 
// - Replace 'your_model_name_ei' with actual Edge Impulse model array/header from your project export.
// - Train the TinyML model on Edge Impulse for 96x96 RGB images detecting red lights (classes: red_light, no_light).
// - For Blynk: Set up app with V0 (Slider 0-255 for speed), V1 (Button for emergency stop).
// - GPS Target: Example coordinates for Islamabad; update as needed.
// - Grid: Simulated 10x10 for A*; expand for larger environments.
// - Calibration: Adjust turn delays for accurate 90° turns based on your chassis.
// - Safety: Emergency button has priority.

#include <NewPing.h>          // Ultrasonic sensor library
#include <TinyGPS++.h>        // GPS parsing library
#include <HardwareSerial.h>   // Serial communication for GPS
#include <BlynkSimpleEsp32.h> // Blynk IoT app integration
#include <EloquentTinyML.h>   // TinyML for AI vision processing
#include "esp_camera.h"       // ESP32-CAM camera library
#include <vector>             // For path storage
#include <queue>              // For A* priority queue
#include <unordered_set>      // For closed set in A*
#include <string>             // For key generation
#include <algorithm>          // For reverse
#include <cmath>              // For abs
#include <cstdlib>            // For abs (int)

using namespace std;

// Pin Definitions
#define LINE_LEFT_PIN 14      // Left IR sensor
#define LINE_CENTER_PIN 15    // Center IR sensor
#define LINE_RIGHT_PIN 27     // Right IR sensor
#define TRIG_PIN 12           // Ultrasonic trigger
#define ECHO_PIN 13           // Ultrasonic echo
#define BUZZER_PIN 32         // Buzzer for alerts
#define EMERGENCY_STOP_PIN 0  // Emergency button (pull-up)
#define MOTOR_IN1 2           // Left motor forward
#define MOTOR_IN2 4           // Left motor backward
#define MOTOR_IN3 5           // Right motor forward
#define MOTOR_IN4 18          // Right motor backward
#define MOTOR_PWM 19          // PWM for speed control

// GPS on UART2 (GPIO 16 RX, 17 TX)
HardwareSerial SerialGPS(2);

// Blynk Authentication and Wi-Fi Credentials (replace with yours)
char blynk_auth[] = "YourBlynkAuthToken";
char wifi_ssid[] = "YourWiFiSSID";
char wifi_pass[] = "YourWiFiPass";

// GPS Target Coordinates (example: Islamabad, Pakistan)
double target_lat = 33.6844;
double target_lon = 73.0479;

// TinyML Model Configuration (from Edge Impulse)
#define EI_CLASSIFIER_RAW_SAMPLES (96 * 96 * 3)  // 96x96 RGB image
#define EI_CLASSIFIER_RAW_SAMPLE_COUNT EI_CLASSIFIER_RAW_SAMPLES
Eloquent::TinyML::TfLite<EI_CLASSIFIER_RAW_SAMPLE_COUNT, 2> ml_model; // 2 classes: 0=red_light, 1=no_light
float vision_features[EI_CLASSIFIER_RAW_SAMPLE_COUNT]; // Input buffer for model features

NewPing sonar(TRIG_PIN, ECHO_PIN, 200); // Ultrasonic with max distance 200cm
TinyGPSPlus gps;                        // GPS object

// Position Structure and Direction Enum for Pathfinding
struct Position {
    int x, y;
    bool operator==(const Position& other) const { return x == other.x && y == other.y; }
};
enum class Direction { NORTH, EAST, SOUTH, WEST };

// Advanced A* PathFinder Class (with dynamic obstacle addition)
class PathFinder {
private:
    const int gridSize = 10;                    // Simulated grid size (adjustable)
    vector<vector<bool>> grid;                  // Grid map: true = obstacle

public:
    PathFinder() : grid(gridSize, vector<bool>(gridSize, false)) {}

    // Add an obstacle to the grid
    void addObstacle(const Position& p) {
        if (p.x >= 0 && p.x < gridSize && p.y >= 0 && p.y < gridSize) {
            grid[p.x][p.y] = true;
        }
    }

    // Manhattan distance heuristic
    int heuristic(const Position& a, const Position& b) {
        return abs(a.x - b.x) + abs(a.y - b.y);
    }

    // Find shortest path using A* algorithm
    vector<Position> findPath(const Position& start, const Position& goal) {
        struct Node {
            Position pos;
            int g, f;
            Node* parent;
            Node(Position p, int gg, int ff, Node* par) : pos(p), g(gg), f(ff), parent(par) {}
        };

        struct Compare {
            bool operator()(Node* a, Node* b) { return a->f > b->f; }
        };

        priority_queue<Node*, vector<Node*>, Compare> openSet;
        unordered_set<string> closedSet;
        openSet.push(new Node(start, 0, heuristic(start, goal), nullptr));

        while (!openSet.empty()) {
            Node* current = openSet.top();
            openSet.pop();

            string key = to_string(current->pos.x) + "," + to_string(current->pos.y);
            if (closedSet.count(key)) continue;
            closedSet.insert(key);

            if (current->pos == goal) {
                // Reconstruct path
                vector<Position> path;
                Node* temp = current;
                while (temp) {
                    path.push_back(temp->pos);
                    temp = temp->parent;
                }
                reverse(path.begin(), path.end());
                // Clean up memory
                delete current;
                while (!openSet.empty()) {
                    delete openSet.top();
                    openSet.pop();
                }
                return path;
            }

            // Neighbors: right, left, down, up
            vector<Position> neighbors = {
                {current->pos.x + 1, current->pos.y},
                {current->pos.x - 1, current->pos.y},
                {current->pos.x, current->pos.y + 1},
                {current->pos.x, current->pos.y - 1}
            };

            for (const auto& n : neighbors) {
                if (n.x < 0 || n.x >= gridSize || n.y < 0 || n.y >= gridSize || grid[n.x][n.y]) continue;

                string nkey = to_string(n.x) + "," + to_string(n.y);
                if (closedSet.count(nkey)) continue;

                int g = current->g + 1;
                int h = heuristic(n, goal);
                openSet.push(new Node(n, g, g + h, current));
            }

            delete current; // Clean up popped node
        }

        // Clean up remaining nodes if no path found
        while (!openSet.empty()) {
            delete openSet.top();
            openSet.pop();
        }

        return {}; // No path
    }
};

PathFinder pathFinder; // Global PathFinder instance

// SelfDrivingRobot Class: Manages all robot functionalities
class SelfDrivingRobot {
private:
    Position currentPos = {0, 0}; // Current simulated grid position
    Direction facing = Direction::EAST; // Initial facing direction

public:
    // Move forward with speed control
    void moveForward(int speed = 200) {
        analogWrite(MOTOR_PWM, speed);
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        digitalWrite(MOTOR_IN3, HIGH);
        digitalWrite(MOTOR_IN4, LOW);
        // Update position based on direction
        switch (facing) {
            case Direction::NORTH: currentPos.y--; break;
            case Direction::EAST: currentPos.x++; break;
            case Direction::SOUTH: currentPos.y++; break;
            case Direction::WEST: currentPos.x--; break;
        }
    }

    // Stop all motors
    void stop() {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, LOW);
    }

    // Turn left 90 degrees
    void turnLeft() {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        digitalWrite(MOTOR_IN3, HIGH);
        digitalWrite(MOTOR_IN4, LOW);
        delay(500); // Adjust for 90° turn calibration
        stop();
        // Update facing (counterclockwise)
        facing = static_cast<Direction>((static_cast<int>(facing) + 3) % 4);
    }

    // Turn right 90 degrees
    void turnRight() {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, HIGH);
        delay(500); // Adjust for 90° turn calibration
        stop();
        // Update facing (clockwise)
        facing = static_cast<Direction>((static_cast<int>(facing) + 1) % 4);
    }

    // Detect obstacle within 20cm
    bool detectObstacle() {
        int distance = sonar.ping_cm();
        if (distance > 0 && distance < 20) {
            tone(BUZZER_PIN, 1000, 200); // Beep alert
            return true;
        }
        return false;
    }

    // Line following logic (assumes LOW = on black line)
    void followLine() {
        int left = digitalRead(LINE_LEFT_PIN);
        int center = digitalRead(LINE_CENTER_PIN);
        int right = digitalRead(LINE_RIGHT_PIN);

        if (center == LOW) {
            moveForward();
        } else if (left == LOW) {
            turnRight();
        } else if (right == LOW) {
            turnLeft();
        } else {
            stop();
        }
    }

    // Avoid obstacle using A* rerouting
    void avoidObstacle() {
        stop();
        // Calculate obstacle position based on facing
        Position obs = currentPos;
        switch (facing) {
            case Direction::NORTH: obs.y--; break;
            case Direction::EAST: obs.x++; break;
            case Direction::SOUTH: obs.y++; break;
            case Direction::WEST: obs.x--; break;
        }
        pathFinder.addObstacle(obs);

        // Find new path to example goal (adjust goal as needed)
        Position goal = {9, 9};
        vector<Position> newPath = pathFinder.findPath(currentPos, goal);

        if (newPath.empty()) {
            Serial.println("No alternative path found!");
            return;
        }

        // Follow the path step by step
        for (size_t i = 1; i < newPath.size(); ++i) {
            Position next = newPath[i];
            int dx = next.x - currentPos.x;
            int dy = next.y - currentPos.y;

            // Determine target direction
            Direction targetDir;
            if (dx == 1) targetDir = Direction::EAST;
            else if (dx == -1) targetDir = Direction::WEST;
            else if (dy == 1) targetDir = Direction::SOUTH;
            else if (dy == -1) targetDir = Direction::NORTH;
            else continue;

            // Turn until facing target direction (minimal turns)
            while (facing != targetDir) {
                // Decide left or right turn for shortest path
                int diff = (static_cast<int>(targetDir) - static_cast<int>(facing) + 4) % 4;
                if (diff == 1 || diff == -3) turnRight();
                else turnLeft();
            }

            // Move one grid step
            moveForward();
            delay(1000); // Adjust based on actual grid step distance/time
            stop();
            currentPos = next;
        }
    }

    // Detect red light using TinyML on camera image
    bool detectRedLight() {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            return false;
        }

        // Preprocess image: Convert RGB565 to normalized RGB floats [0,1]
        uint8_t* buf = fb->buf;
        int index = 0;
        for (size_t i = 0; i < fb->len; i += 2) {
            uint16_t pixel = (buf[i + 1] << 8) | buf[i]; // Little-endian RGB565
            float r = ((pixel >> 11) & 0x1F) / 31.0f;
            float g = ((pixel >> 5) & 0x3F) / 63.0f;
            float b = (pixel & 0x1F) / 31.0f;
            vision_features[index++] = r;
            vision_features[index++] = g;
            vision_features[index++] = b;
        }

        // Predict using model: 0 = red_light, 1 = no_light
        int prediction = ml_model.predict(vision_features);
        esp_camera_fb_return(fb);

        if (prediction == 0) {
            tone(BUZZER_PIN, 1500, 300); // Alert tone
            return true;
        }
        return false;
    }

    // GPS navigation to target
    void navigateGPS() {
        while (SerialGPS.available() > 0) {
            gps.encode(SerialGPS.read());
        }

        if (gps.location.isValid()) {
            double distance = TinyGPSPlus::distanceBetween(
                gps.location.lat(), gps.location.lng(), target_lat, target_lon);
            Serial.printf("Distance to target: %.2f meters\n", distance);

            if (distance < 5.0) {
                stop();
                Serial.println("Target reached!");
            } else {
                // Basic: Move forward (add compass for bearing adjustments in advanced setup)
                moveForward();
            }
        } else {
            Serial.println("Waiting for GPS fix...");
        }
    }
};

SelfDrivingRobot robot; // Global robot instance

// Blynk Virtual Pin Handlers
BLYNK_WRITE(V0) { // Speed control slider (0-255)
    robot.moveForward(param.asInt());
}

BLYNK_WRITE(V1) { // Emergency stop button (1 = stop)
    if (param.asInt() == 1) {
        robot.stop();
        Serial.println("Blynk app emergency stop triggered!");
    }
}

void setup() {
    Serial.begin(115200);
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // Initialize GPS UART

    // Pin modes
    pinMode(LINE_LEFT_PIN, INPUT);
    pinMode(LINE_CENTER_PIN, INPUT);
    pinMode(LINE_RIGHT_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

    // ESP32-CAM Initialization
    camera_config_t config;
    config.pin_d0 = 5;
    config.pin_d1 = 18;
    config.pin_d2 = 19;
    config.pin_d3 = 21;
    config.pin_d4 = 36;
    config.pin_d5 = 39;
    config.pin_d6 = 34;
    config.pin_d7 = 35;
    config.pin_xclk = 0;
    config.pin_pclk = 22;
    config.pin_vsync = 25;
    config.pin_href = 23;
    config.pin_sscb_sda = 26;
    config.pin_sscb_scl = 27;
    config.pin_pwdn = 32;
    config.pin_reset = -1;
    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.pixel_format = PIXFORMAT_RGB565; // Matches TinyML input
    config.frame_size = FRAMESIZE_96X96;    // Model input size
    config.jpeg_quality = 12;               // Not used for RGB
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera initialization failed with error 0x%x\n", err);
        return; // Halt if camera fails
    }

    // Blynk and Wi-Fi connection
    Blynk.begin(blynk_auth, wifi_ssid, wifi_pass);

    // Initialize TinyML model (replace with your Edge Impulse model)
    ml_model.begin(your_model_name_ei);

    Serial.println("Setup complete. Robot ready.");
}

void loop() {
    Blynk.run(); // Process Blynk app events

    // Priority: Check emergency stop button
    if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
        robot.stop();
        tone(BUZZER_PIN, 2000, 1000); // Loud alarm
        delay(200); // Debounce
        Serial.println("Hardware emergency stop triggered!");
        return; // Skip other actions
    }

    // Check for red light via AI vision
    if (robot.detectRedLight()) {
        robot.stop();
        Serial.println("AI Vision: Red light detected - stopping");
        delay(5000); // Simulate waiting for green (add logic for detection if needed)
        return;
    }

    // Check for obstacles
    if (robot.detectObstacle()) {
        robot.avoidObstacle();
        return;
    }

    // Default autonomous behaviors
    robot.followLine();
    robot.navigateGPS();

    delay(10); // Small delay for loop stability
}