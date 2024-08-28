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

// Timing
unsigned long previousMillis = 0;
const long interval = 30;  // Adjusted time between each step

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

        // Smooth pan movement with smaller step
        if (panAngle != targetPanAngle) {
            if (abs(panAngle - targetPanAngle) > 0.5) {
                panAngle += (panAngle < targetPanAngle) ? 0.5 : -0.5; // Smaller step size
            } else {
                panAngle = targetPanAngle; // Snap to target if very close
            }
            pwm.setPWM(servoPanChannel, 0, angleToPulse(panAngle, false));
        }

        // Smooth tilt movement with limiter
        if (tiltAngle != targetTiltAngle) {
            float constrainedTilt = constrain(targetTiltAngle, 70, 160); // Limit downward tilt to 70 degrees
            if (tiltAngle != constrainedTilt) {
                tiltAngle += (tiltAngle < constrainedTilt) ? 1 : -1;
                pwm.setPWM(servoTiltChannel, 0, angleToPulse(tiltAngle, true));
            }
        }

        // Smooth right ear movement
        if (rightEarAngle != targetRightEarAngle) {
            rightEarAngle += (rightEarAngle < targetRightEarAngle) ? 1 : -1;
            pwm.setPWM(servoRightEarChannel, 0, rightEarAngleToPulse(rightEarAngle));
        }

        // Smooth left ear movement
        if (leftEarAngle != targetLeftEarAngle) {
            leftEarAngle += (leftEarAngle < targetLeftEarAngle) ? 1 : -1;
            pwm.setPWM(servoLeftEarChannel, 0, leftEarAngleToPulse(leftEarAngle));
        }

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
        html += "<p>Right Ear Angle: <span id='rightEarAngle'>" + String(rightEarAngle) + "</span></p>";
        html += "<p>Left Ear Angle: <span id='leftEarAngle'>" + String(leftEarAngle) + "</span></p>";
        html += "<button class='btn' onmousedown=\"fetch('/down')\" onmouseup=\"fetch('/stop')\">Tilt Head Down</button>";
        html += "<button class='btn' onmousedown=\"fetch('/up')\" onmouseup=\"fetch('/stop')\">Tilt Head Up</button>";
        html += "<button class='btn' onmousedown=\"fetch('/left')\" onmouseup=\"fetch('/stop')\">Pan Left</button>";
        html += "<button class='btn' onmousedown=\"fetch('/right')\" onmouseup=\"fetch('/stop')\">Pan Right</button>";
        html += "<button class='btn' onclick=\"fetch('/center')\">Center</button>";
        html += "<button class='btn' onclick=\"fetch('/ninety')\">90</button>";
        html += "<button class='btn' onmousedown=\"fetch('/right_ear_up')\" onmouseup=\"fetch('/stop')\">Right Ear Up</button>";
        html += "<button class='btn' onmousedown=\"fetch('/right_ear_down')\" onmouseup=\"fetch('/stop')\">Right Ear Down</button>";
        html += "<button class='btn' onmousedown=\"fetch('/left_ear_up')\" onmouseup=\"fetch('/stop')\">Left Ear Up</button>";
        html += "<button class='btn' onmousedown=\"fetch('/left_ear_down')\" onmouseup=\"fetch('/stop')\">Left Ear Down</button>";
        html += "<button class='btn' onclick=\"fetch('/demo')\">Demo</button>";
        html += "</div>";
        html += "<script>";
        html += "function updateAngles() {";
        html += "fetch('/angles').then(response => response.json()).then(data => {";
        html += "document.getElementById('panAngle').innerText = data.pan;";
        html += "document.getElementById('tiltAngle').innerText = data.tilt;";
        html += "document.getElementById('rightEarAngle').innerText = data.rightEar;";
        html += "document.getElementById('leftEarAngle').innerText = data.leftEar;";
        html += "});";
        html += "}";
        html += "setInterval(updateAngles, 1000);";  // Update angles every second
        html += "</script>";
        html += "</body></html>";
        request->send(200, "text/html", html);
    });

    // API endpoint to return current angles
    server.on("/angles", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "{";
        json += "\"pan\":" + String(panAngle) + ",";
        json += "\"tilt\":" + String(tiltAngle) + ",";
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

    server.on("/ninety", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(90, 50, -10, -10);
        request->send(204); // No content response
    });

    server.on("/right_ear_up", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(targetPanAngle, targetTiltAngle, targetRightEarAngle + 1, targetLeftEarAngle);
        request->send(204); // No content response
    });

    server.on("/right_ear_down", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(targetPanAngle, targetTiltAngle, targetRightEarAngle - 1, targetLeftEarAngle);
        request->send(204); // No content response
    });

    server.on("/left_ear_up", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(targetPanAngle, targetTiltAngle, targetRightEarAngle, targetLeftEarAngle - 1);
        request->send(204); // No content response
    });

    server.on("/left_ear_down", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(targetPanAngle, targetTiltAngle, targetRightEarAngle, targetLeftEarAngle + 1);
        request->send(204); // No content response
    });

    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
        isMoving = false; // Stop all movement
        request->send(204); // No content response
    });

    // Handle Demo command
    server.on("/demo", HTTP_GET, [](AsyncWebServerRequest *request){
        // Center head and set initial ear positions
        startMovement(120, 90, -15, -5);
        delay(1000);

        // Wiggle ears
        for (int i = 0; i < 3; i++) {
            startMovement(120, 90, -10, -10);
            delay(500);
            startMovement(120, 90, -15, -5);
            delay(500);
        }

        // Move ears to opposite positions
        startMovement(120, 90, -5, -15);
        delay(1000);

        // Pan head from side to side
        for (int i = 0; i < 3; i++) {
            startMovement(200, 90, rightEarAngle, leftEarAngle);
            isMoving = true;
            while (isMoving) updateMovement();
            delay(1000);
            
            startMovement(0, 90, rightEarAngle, leftEarAngle);
            isMoving = true;
            while (isMoving) updateMovement();
            delay(1000);
        }

        // Pan head while tilting up and down
        for (int i = 0; i < 2; i++) {
            startMovement(200, 70, rightEarAngle, leftEarAngle);
            isMoving = true;
            while (isMoving) updateMovement();
            delay(1000);
            
            startMovement(0, 112, rightEarAngle, leftEarAngle);
            isMoving = true;
            while (isMoving) updateMovement();
            delay(1000);
        }

        // Reset to center
        startMovement(120, 90, -10, -10);
        isMoving = true;
        while (isMoving) updateMovement();
        request->send(204); // No content response
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
