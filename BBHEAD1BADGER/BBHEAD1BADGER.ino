#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Wi-Fi credentials
const char* ssid = "SpectrumSetup-DD";
const char* password = "jeansrocket543";

// Create PWM servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo channels
int servoPanChannel = 0;
int servoTiltChannel = 1;
int servoRightEarChannel = 2;
int servoLeftEarChannel = 3;

// Positions
float panAngle = 120;  // New centered pan (looking forward)
float tiltAngle = 90;  // New centered tilt (looking forward)
float rightEarAngle = 0;  // Start right ear at 0
float leftEarAngle = -20;  // Start left ear at -20
float targetPanAngle = 120;
float targetTiltAngle = 90;
float targetRightEarAngle = -10;  // "Ready" position
float targetLeftEarAngle = -10;   // "Ready" position
bool isMoving = false;

// Speed and acceleration settings
float panSpeed = 2;   // Adjust the pan speed dynamically
float tiltSpeed = 2;  // Adjust the tilt speed dynamically
float panAcceleration = 0.1;  // Adjust pan acceleration
float tiltAcceleration = 0.1; // Adjust tilt acceleration

// Timing
unsigned long previousMillis = 0;
const long interval = 20;  // Adjusted time between each step

// Adjusted Pulse Width Range
#define SERVOMIN  100  // Minimum pulse length out of 4096 (adjusted for MG995)
#define SERVOMAX  650  // Maximum pulse length out of 4096 (adjusted for MG995)

// Web server
AsyncWebServer server(80);

// Function to convert angle to pulse length for the right ear
uint16_t rightEarAngleToPulse(float angle) {
    return map(constrain(angle, -21, 1), -21, 1, 150, 600); // Original MG90S pulse range
}

// Function to convert angle to pulse length for the left ear
uint16_t leftEarAngleToPulse(float angle) {
    return map(constrain(angle, -21, 1), -21, 1, 150, 600); // Original MG90S pulse range
}

// Function to convert angle to pulse length for pan/tilt (MG995 servos)
uint16_t angleToPulse(float angle, bool isTilt) {
    if (isTilt) {
        return map(angle, 50, 160, SERVOMIN, SERVOMAX);  // Tilt range: 50 (up) to 160 (down)
    } else {
        return map(angle, -10, 200, SERVOMIN, SERVOMAX); // Pan range: -10 to 200
    }
}

// Function to start smooth movement
void startMovement(float newTargetPan, float newTargetTilt, float newTargetRightEar, float newTargetLeftEar) {
    targetPanAngle = newTargetPan;
    targetTiltAngle = newTargetTilt;
    targetRightEarAngle = newTargetRightEar;
    targetLeftEarAngle = newTargetLeftEar;
    isMoving = true;
}

// Function for non-blocking smooth movement
void updateMovement() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Smooth pan movement with adjustable speed and acceleration
        if (panAngle != targetPanAngle) {
            float step = panSpeed * panAcceleration;
            if (abs(panAngle - targetPanAngle) > step) {
                panAngle += (panAngle < targetPanAngle) ? step : -step;
            } else {
                panAngle = targetPanAngle; // Snap to target if very close
            }
            pwm.setPWM(servoPanChannel, 0, angleToPulse(panAngle, false));
        }

        // Smooth tilt movement with adjustable speed and acceleration
        if (tiltAngle != targetTiltAngle) {
            float constrainedTilt = constrain(targetTiltAngle, 70, 160); // Limit downward tilt to 70 degrees
            float step = tiltSpeed * tiltAcceleration;
            if (abs(tiltAngle - constrainedTilt) > step) {
                tiltAngle += (tiltAngle < constrainedTilt) ? step : -step;
            } else {
                tiltAngle = constrainedTilt; // Snap to target if very close
            }
            pwm.setPWM(servoTiltChannel, 0, angleToPulse(tiltAngle, true));
        }

        // Smooth right ear movement
        if (rightEarAngle != targetRightEarAngle) {
            rightEarAngle += (rightEarAngle < targetRightEarAngle) ? 2 : -2;
            pwm.setPWM(servoRightEarChannel, 0, rightEarAngleToPulse(rightEarAngle));
        }

        // Smooth left ear movement
        if (leftEarAngle != targetLeftEarAngle) {
            leftEarAngle += (leftEarAngle < targetLeftEarAngle) ? 2 : -2;
            pwm.setPWM(servoLeftEarChannel, 0, leftEarAngleToPulse(leftEarAngle));
        }

        // Stop movement when all targets are reached
        if (panAngle == targetPanAngle && tiltAngle == targetTiltAngle && rightEarAngle == targetRightEarAngle && leftEarAngle == targetLeftEarAngle) {
            isMoving = false;
        }
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Initialize PWM driver with a slightly higher frequency for MG995 servos
    pwm.begin();
    pwm.setPWMFreq(65);  // Increase PWM frequency slightly for better MG995 response

    // Connect to Wi-Fi
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Move to initial position on startup
    startMovement(120, 90, 0, -20);
    Serial.println("Moved to initial position on startup (Pan: 120°, Tilt: 90°, Right Ear: 0°, Left Ear: -20°).");

    // Define web server routes
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = "<html><head><style>";
        html += "body { font-family: Arial, sans-serif; text-align: center; margin: 0; padding: 0; }";
        html += ".container { max-width: 300px; margin: auto; }";
        html += ".btn { display: inline-block; padding: 15px; margin: 5px; font-size: 16px; cursor: pointer; background-color: #008CBA; color: white; border: none; border-radius: 5px; }";
        html += ".btn:active { background-color: #005f73; }";
        html += "</style></head><body>";
        html += "<div class='container'>";
        html += "<h1>BB-1 Head Control</h1>";
        html += "<p>Pan Angle: <span id='panAngle'>" + String(panAngle) + "</span></p>";
        html += "<p>Tilt Angle: <span id='tiltAngle'>" + String(tiltAngle) + "</span></p>";
        html += "<p>Pan Speed: <span id='panSpeed'>" + String(panSpeed) + "</span></p>";
        html += "<p>Tilt Speed: <span id='tiltSpeed'>" + String(tiltSpeed) + "</span></p>";
        html += "<p>Pan Acceleration: <span id='panAcceleration'>" + String(panAcceleration) + "</span></p>";
        html += "<p>Tilt Acceleration: <span id='tiltAcceleration'>" + String(tiltAcceleration) + "</span></p>";
        html += "<button class='btn' onclick=\"fetch('/increase_pan_speed')\">Increase Pan Speed</button>";
        html += "<button class='btn' onclick=\"fetch('/decrease_pan_speed')\">Decrease Pan Speed</button>";
        html += "<button class='btn' onclick=\"fetch('/increase_tilt_speed')\">Increase Tilt Speed</button>";
        html += "<button class='btn' onclick=\"fetch('/decrease_tilt_speed')\">Decrease Tilt Speed</button>";
        html += "<button class='btn' onclick=\"fetch('/increase_pan_acceleration')\">Increase Pan Acceleration</button>";
        html += "<button class='btn' onclick=\"fetch('/decrease_pan_acceleration')\">Decrease Pan Acceleration</button>";
        html += "<button class='btn' onclick=\"fetch('/increase_tilt_acceleration')\">Increase Tilt Acceleration</button>";
        html += "<button class='btn' onclick=\"fetch('/decrease_tilt_acceleration')\">Decrease Tilt Acceleration</button>";
        html += "<button class='btn' onmousedown=\"fetch('/up')\" onmouseup=\"fetch('/stop')\">Tilt Head Up</button>";
        html += "<button class='btn' onmousedown=\"fetch('/down')\" onmouseup=\"fetch('/stop')\">Tilt Head Down</button>";
        html += "<button class='btn' onmousedown=\"fetch('/left')\" onmouseup=\"fetch('/stop')\">Pan Left</button>";
        html += "<button class='btn' onmousedown=\"fetch('/right')\" onmouseup=\"fetch('/stop')\">Pan Right</button>";
        html += "<button class='btn' onclick=\"fetch('/center')\">Center</button>";
        html += "</div>";
        html += "<script>";
        html += "function updateAngles() {";
        html += "fetch('/angles').then(response => response.json()).then(data => {";
        html += "document.getElementById('panAngle').innerText = data.pan;";
        html += "document.getElementById('tiltAngle').innerText = data.tilt;";
        html += "document.getElementById('panSpeed').innerText = data.panSpeed;";
        html += "document.getElementById('tiltSpeed').innerText = data.tiltSpeed;";
        html += "document.getElementById('panAcceleration').innerText = data.panAcceleration;";
        html += "document.getElementById('tiltAcceleration').innerText = data.tiltAcceleration;";
        html += "});";
        html += "}";
        html += "setInterval(updateAngles, 1000);";  // Update angles every second
        html += "</script>";
        html += "</body></html>";
        request->send(200, "text/html", html);
    });

    // API endpoint to return current angles, speeds, and accelerations
    server.on("/angles", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "{";
        json += "\"pan\":" + String(panAngle) + ",";
        json += "\"tilt\":" + String(tiltAngle) + ",";
        json += "\"panSpeed\":" + String(panSpeed) + ",";
        json += "\"tiltSpeed\":" + String(tiltSpeed) + ",";
        json += "\"panAcceleration\":" + String(panAcceleration) + ",";
        json += "\"tiltAcceleration\":" + String(tiltAcceleration) + ",";
        json += "\"rightEar\":" + String(rightEarAngle) + ",";
        json += "\"leftEar\":" + String(leftEarAngle);
        json += "}";
        request->send(200, "application/json", json);
    });

    // Movement control handlers
    server.on("/up", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(targetPanAngle, constrain(targetTiltAngle + 10, 70, 160), targetRightEarAngle, targetLeftEarAngle);
        request->send(204); // No content response
    });

    server.on("/down", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(targetPanAngle, constrain(targetTiltAngle - 10, 70, 160), targetRightEarAngle, targetLeftEarAngle);
        request->send(204); // No content response
    });

    server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(constrain(targetPanAngle + 10, -10, 200), targetTiltAngle, targetRightEarAngle, targetLeftEarAngle);
        request->send(204); // No content response
    });

    server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(constrain(targetPanAngle - 10, -10, 200), targetTiltAngle, targetRightEarAngle, targetLeftEarAngle);
        request->send(204); // No content response
    });

    server.on("/center", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(120, 90, -10, -10);  // Updated center positions
        request->send(204); // No content response
    });

    // Speed and Acceleration Control Endpoints
    server.on("/increase_pan_speed", HTTP_GET, [](AsyncWebServerRequest *request){
        panSpeed += 0.5;
        request->send(204);
    });

    server.on("/decrease_pan_speed", HTTP_GET, [](AsyncWebServerRequest *request){
        panSpeed = max(0.5, panSpeed - 0.5);  // Minimum speed is 0.5
        request->send(204);
    });

    server.on("/increase_tilt_speed", HTTP_GET, [](AsyncWebServerRequest *request){
        tiltSpeed += 0.5;
        request->send(204);
    });

    server.on("/decrease_tilt_speed", HTTP_GET, [](AsyncWebServerRequest *request){
        tiltSpeed = max(0.5, tiltSpeed - 0.5);  // Minimum speed is 0.5
        request->send(204);
    });

    server.on("/increase_pan_acceleration", HTTP_GET, [](AsyncWebServerRequest *request){
        panAcceleration += 0.05;
        request->send(204);
    });

    server.on("/decrease_pan_acceleration", HTTP_GET, [](AsyncWebServerRequest *request){
        panAcceleration = max(0.05, panAcceleration - 0.05);  // Minimum acceleration is 0.05
        request->send(204);
    });

    server.on("/increase_tilt_acceleration", HTTP_GET, [](AsyncWebServerRequest *request){
        tiltAcceleration += 0.05;
        request->send(204);
    });

    server.on("/decrease_tilt_acceleration", HTTP_GET, [](AsyncWebServerRequest *request){
        tiltAcceleration = max(0.05, tiltAcceleration - 0.05);  // Minimum acceleration is 0.05
        request->send(204);
    });

    // Start server
    server.begin();
    Serial.println("Server started.");
}

void loop() {
    // Non-blocking update of movement
    if (isMoving) {
        updateMovement();
    }
}
