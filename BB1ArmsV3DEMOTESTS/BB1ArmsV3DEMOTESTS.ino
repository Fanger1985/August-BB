#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WebServer.h>

// Initialize the PCA9685
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// WiFi credentials
const char* ssid = "SpectrumSetup-DD";
const char* password = "jeansrocket543";

// Web server running on port 80
WebServer server(80);

#define SERVOMIN  150 // Minimum pulse length out of 4096
#define SERVOMAX  600 // Maximum pulse length out of 4096

bool autoMode = false;

// Current angles of the servos
uint16_t servoAnglesLeft[5] = {90, 90, 90, 90, 90}; // Initial angles for left shoulder, wrist, and claw servos
uint16_t servoAnglesRight[5] = {90, 90, 90, 90, 90}; // Initial angles for right shoulder, wrist, and claw servos

// Function to set servo angle
void setServoAngle(uint8_t channel, uint16_t angle) {
    uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pca.setPWM(channel, 0, pulse);

    // Update the current angle for the servo
    if (channel < 5) {
        servoAnglesLeft[channel] = angle;
    } else if (channel >= 4 && channel < 9) {
        servoAnglesRight[channel - 4] = angle;
    } else if (channel == 15) {
        servoAnglesLeft[4] = angle;
    } else if (channel == 14) {
        servoAnglesRight[4] = angle;
    }
}

// Function for smooth movement
void moveServoSmooth(uint8_t channel, uint16_t startAngle, uint16_t endAngle, uint16_t steps, uint16_t delayTime) {
    float stepSize = (endAngle - startAngle) / (float)steps;
    for (uint16_t i = 0; i <= steps; i++) {
        setServoAngle(channel, startAngle + stepSize * i);
        delay(delayTime);
    }
}

// Handle root URL "/"
void handleRoot() {
    String html = "<html>\
    <head>\
    <title>Servo Control</title>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <style>\
    body { font-family: Arial, sans-serif; text-align: center; }\
    .button { padding: 10px 20px; margin: 5px; font-size: 20px; }\
    .slider { width: 90%; }\
    .panel { display: inline-block; width: 45%; vertical-align: top; }\
    .left-panel { float: left; }\
    .right-panel { float: right; }\
    </style>\
    </head>\
    <body>\
    <h1>Control Servos</h1>\
    <button class='button' onclick=\"sendRequest('/calibrate')\">Calibrate</button>\
    <button class='button' onclick=\"sendRequest('/demo')\">Demo</button>\
    <button class='button' onclick=\"sendRequest('/demo2')\">Demo 2</button>\
    <button class='button' onclick=\"sendRequest('/demo3')\">Demo 3</button>\
    <button class='button' onclick=\"sendRequest('/printAngles')\">Print Angles</button>\
    <button class='button' onclick=\"sendRequest('/showDemo')\">Show Demo</button>\
    <div class='panel left-panel'>\
    <h2>Left Arm</h2>\
    <h3>Shoulder Servo A</h3>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo0' onchange=\"updateServo(0, this.value)\">\
    <span id='angle0'>90</span>\
    <h3>Shoulder Servo B</h3>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo1' onchange=\"updateServo(1, this.value)\">\
    <span id='angle1'>90</span>\
    <h3>Wrist Servo A</h3>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo2' onchange=\"updateServo(2, this.value)\">\
    <span id='angle2'>90</span>\
    <h3>Wrist Servo B</h3>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo3' onchange=\"updateServo(3, this.value)\">\
    <span id='angle3'>90</span>\
    <h3>Claw</h3>\
    <input type='range' min='55' max='176' value='90' class='slider' id='servo15' onchange=\"updateServo(15, this.value)\">\
    <span id='angle15'>90</span>\
    </div>\
    <div class='panel right-panel'>\
    <h2>Right Arm</h2>\
    <h3>Shoulder Servo A</h3>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo4' onchange=\"updateServo(4, this.value)\">\
    <span id='angle4'>90</span>\
    <h3>Shoulder Servo B</h3>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo5' onchange=\"updateServo(5, this.value)\">\
    <span id='angle5'>90</span>\
    <h3>Wrist Servo A</h3>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo6' onchange=\"updateServo(6, this.value)\">\
    <span id='angle6'>90</span>\
    <h3>Wrist Servo B</h3>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo7' onchange=\"updateServo(7, this.value)\">\
    <span id='angle7'>90</span>\
    <h3>Claw</h3>\
    <input type='range' min='55' max='176' value='90' class='slider' id='servo14' onchange=\"updateServo(14, this.value)\">\
    <span id='angle14'>90</span>\
    </div>\
    <script>\
    function sendRequest(endpoint) {\
        fetch(endpoint).then(response => console.log(response));\
    }\
    function updateServo(channel, angle) {\
        fetch(`/moveServo?channel=${channel}&angle=${angle}`).then(response => {\
            if (response.ok) {\
                document.getElementById(`angle${channel}`).innerText = angle;\
            }\
        });\
    }\
    function updateSliders(anglesLeft, anglesRight) {\
        document.getElementById('servo0').value = anglesLeft[0];\
        document.getElementById('angle0').innerText = anglesLeft[0];\
        document.getElementById('servo1').value = anglesLeft[1];\
        document.getElementById('angle1').innerText = anglesLeft[1];\
        document.getElementById('servo2').value = anglesLeft[2];\
        document.getElementById('angle2').innerText = anglesLeft[2];\
        document.getElementById('servo3').value = anglesLeft[3];\
        document.getElementById('angle3').innerText = anglesLeft[3];\
        document.getElementById('servo15').value = anglesLeft[4];\
        document.getElementById('angle15').innerText = anglesLeft[4];\
        document.getElementById('servo4').value = anglesRight[0];\
        document.getElementById('angle4').innerText = anglesRight[0];\
        document.getElementById('servo5').value = anglesRight[1];\
        document.getElementById('angle5').innerText = anglesRight[1];\
        document.getElementById('servo6').value = anglesRight[2];\
        document.getElementById('angle6').innerText = anglesRight[2];\
        document.getElementById('servo7').value = anglesRight[3];\
        document.getElementById('angle7').innerText = anglesRight[3];\
        document.getElementById('servo14').value = anglesRight[4];\
        document.getElementById('angle14').innerText = anglesRight[4];\
    }\
    fetch('/getAngles').then(response => response.json()).then(data => updateSliders(data.left, data.right));\
    </script>\
    </body>\
    </html>";

    server.send(200, "text/html", html);
}

// Handle URL "/moveServo"
void handleMoveServo() {
    if (server.hasArg("channel") && server.hasArg("angle")) {
        uint8_t channel = server.arg("channel").toInt();
        uint16_t angle = server.arg("angle").toInt();
        setServoAngle(channel, angle);
        server.send(200, "text/plain", "Moved servo");
    } else {
        server.send(400, "text/plain", "Invalid request");
    }
}

// Handle URL "/getAngles"
void handleGetAngles() {
    String json = "{\"left\":[";
    json += String(servoAnglesLeft[0]) + ",";
    json += String(servoAnglesLeft[1]) + ",";
    json += String(servoAnglesLeft[2]) + ",";
    json += String(servoAnglesLeft[3]) + ",";
    json += String(servoAnglesLeft[4]);
    json += "], \"right\":[";
    json += String(servoAnglesRight[0]) + ",";
    json += String(servoAnglesRight[1]) + ",";
    json += String(servoAnglesRight[2]) + ",";
    json += String(servoAnglesRight[3]) + ",";
    json += String(servoAnglesRight[4]);
    json += "]}";
    server.send(200, "application/json", json);
}

// Handle URL "/calibrate"
void handleCalibrate() {
    for (uint8_t i = 0; i < 5; i++) {
        setServoAngle(i, 90);
        setServoAngle(i + 4, 90);
    }
    server.send(200, "text/plain", "Servos calibrated to 90 degrees");
}

// Handle URL "/printAngles"
void handlePrintAngles() {
    printCurrentAngles();
    server.send(200, "text/plain", "Printed current angles to serial monitor");
}

// Function to print current angles to the serial monitor
void printCurrentAngles() {
    Serial.println("Current servo angles:");
    for (uint8_t i = 0; i < 5; i++) {
        Serial.print("Left Servo ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(servoAnglesLeft[i]);
    }
    for (uint8_t i = 0; i < 5; i++) {
        Serial.print("Right Servo ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(servoAnglesRight[i]);
    }
}

void handleDemo() {
    const int steps = 30;
    const int delayTime = 20;

    // Initial pose based on the screenshot
    uint16_t initialLeftAngles[5] = {48, 71, 164, 174, 63};
    uint16_t initialRightAngles[5] = {132, 91, 80, 112, 152};

    uint16_t chestBeatLeftAngles1[5] = {48, 38, 164, 174, 63};
    uint16_t chestBeatRightAngles1[5] = {132, 43, 80, 112, 152};

    uint16_t chestBeatLeftAngles2[5] = {48, 71, 164, 174, 63};
    uint16_t chestBeatRightAngles2[5] = {132, 91, 80, 112, 152};

    uint16_t finalLeftAngles[5] = {4, 159, 164, 173, 97};
    uint16_t finalRightAngles[5] = {168, 149, 87, 162, 65};

    uint16_t wristMoveLeft[2] = {38, 157};
    uint16_t wristMoveRight[2] = {83, 151};

    uint16_t lastLeftAngles[5] = {101, 76, 41, 81, 172};
    uint16_t lastRightAngles[5] = {91, 40, 140, 148, 168};

    // Move to calibration position
    for (uint8_t i = 0; i < 5; i++) {
        setServoAngle(i, 90);
        setServoAngle(i + 4, 90);
    }
    delay(1000); // Small delay after moving to calibration position

    // Move to the initial pose from the screenshot
    for (int i = 0; i < 5; i++) {
        moveServoSmooth(i, 90, initialLeftAngles[i], steps, delayTime);
        moveServoSmooth(i + 4, 90, initialRightAngles[i], steps, delayTime);
    }
    delay(1000); // Small delay before next movement

    // Beat chest back and forth
    for (int rep = 0; rep < 3; rep++) {
        for (int i = 0; i < 5; i++) {
            moveServoSmooth(i, initialLeftAngles[i], chestBeatLeftAngles1[i], steps, delayTime);
            moveServoSmooth(i + 4, initialRightAngles[i], chestBeatRightAngles1[i], steps, delayTime);
        }
        delay(500); // Small delay before reversing
        for (int i = 0; i < 5; i++) {
            moveServoSmooth(i, chestBeatLeftAngles1[i], chestBeatLeftAngles2[i], steps, delayTime);
            moveServoSmooth(i + 4, chestBeatRightAngles1[i], chestBeatRightAngles2[i], steps, delayTime);
        }
        delay(500); // Small delay before next rep
    }

    // Move to the next positions from screenshot 3
    for (int i = 0; i < 5; i++) {
        moveServoSmooth(i, chestBeatLeftAngles2[i], finalLeftAngles[i], steps, delayTime);
        moveServoSmooth(i + 4, chestBeatRightAngles2[i], finalRightAngles[i], steps, delayTime);
    }
    delay(1000); // Small delay before next movement

    // Move to the final positions from screenshot 4
    for (int i = 0; i < 5; i++) {
        moveServoSmooth(i, finalLeftAngles[i], lastLeftAngles[i], steps, delayTime);
        moveServoSmooth(i + 4, finalRightAngles[i], lastRightAngles[i], steps, delayTime);
    }
    delay(1000); // Small delay before next movement

    // Move wrist servos back and forth
    for (int rep = 0; rep < 2; rep++) {
        moveServoSmooth(3, lastLeftAngles[3], wristMoveLeft[0], steps, delayTime);
        moveServoSmooth(7, lastRightAngles[3], wristMoveRight[0], steps, delayTime);
        delay(500); // Small delay before reversing
        moveServoSmooth(3, wristMoveLeft[0], wristMoveLeft[1], steps, delayTime);
        moveServoSmooth(7, wristMoveRight[0], wristMoveRight[1], steps, delayTime);
        delay(500); // Small delay before next rep
    }

    // Move back to the positions from screenshot 2
    for (int i = 0; i < 5; i++) {
        moveServoSmooth(i, lastLeftAngles[i], chestBeatLeftAngles1[i], steps, delayTime);
        moveServoSmooth(i + 4, lastRightAngles[i], chestBeatRightAngles1[i], steps, delayTime);
    }
    delay(1000); // Small delay before next movement

    // Move to calibration position
    for (uint8_t i = 0; i < 5; i++) {
        setServoAngle(i, 90);
        setServoAngle(i + 4, 90);
    }
    server.send(200, "text/plain", "Demo completed");
}

// Function for the "Show Demo"
void handleShowDemo() {
    const int steps = 30;
    const int delayTime = 20;

    // Calibrate to initial position
    handleCalibrate();
    delay(500);

    // POSE 1 (Screenshot 1)
    moveServoSmooth(0, 90, 45, steps, delayTime);
    moveServoSmooth(1, 90, 126, steps, delayTime);
    moveServoSmooth(2, 90, 31, steps, delayTime);
    moveServoSmooth(3, 90, 160, steps, delayTime);
    moveServoSmooth(15, 90, 103, steps, delayTime);
    moveServoSmooth(4, 90, 120, steps, delayTime);
    moveServoSmooth(5, 90, 42, steps, delayTime);
    moveServoSmooth(6, 90, 99, steps, delayTime);
    moveServoSmooth(7, 90, 170, steps, delayTime);
    moveServoSmooth(14, 90, 174, steps, delayTime);
    delay(500);

    // Wrist movements in POSE 1
    for (int i = 0; i < 2; i++) {
        moveServoSmooth(3, 160, 120, steps, delayTime);
        moveServoSmooth(7, 170, 120, steps, delayTime);
        moveServoSmooth(15, 103, 50, steps, delayTime);
        delay(500);
        moveServoSmooth(3, 120, 160, steps, delayTime);
        moveServoSmooth(7, 120, 170, steps, delayTime);
        moveServoSmooth(15, 50, 175, steps, delayTime);
        delay(500);
    }

    // POSE 2 (Screenshot 2)
    moveServoSmooth(0, 45, 104, steps, delayTime);
    moveServoSmooth(1, 126, 81, steps, delayTime);
    moveServoSmooth(2, 31, 116, steps, delayTime);
    moveServoSmooth(3, 160, 175, steps, delayTime);
    moveServoSmooth(15, 103, 157, steps, delayTime);
    moveServoSmooth(4, 120, 100, steps, delayTime);
    moveServoSmooth(5, 42, 96, steps, delayTime);
    moveServoSmooth(6, 99, 154, steps, delayTime);
    moveServoSmooth(7, 170, 175, steps, delayTime);
    moveServoSmooth(14, 174, 173, steps, delayTime);
    delay(500);

    // POSE 3 (Screenshot 3)
    moveServoSmooth(0, 104, 52, steps, delayTime);
    moveServoSmooth(1, 81, 125, steps, delayTime);
    moveServoSmooth(2, 116, 34, steps, delayTime);
    moveServoSmooth(3, 175, 140, steps, delayTime);
    moveServoSmooth(15, 157, 140, steps, delayTime);
    moveServoSmooth(4, 100, 100, steps, delayTime);
    moveServoSmooth(5, 96, 78, steps, delayTime);
    moveServoSmooth(6, 154, 78, steps, delayTime);
    moveServoSmooth(7, 175, 173, steps, delayTime);
    moveServoSmooth(14, 173, 165, steps, delayTime);

    // Moving Shoulder Servo B on right arm during POSE 3
    for (int i = 0; i < 2; i++) {
        moveServoSmooth(5, 78, 60, steps, delayTime);
        delay(500);
        moveServoSmooth(5, 60, 116, steps, delayTime);
        delay(500);
    }

    // POSE 4 (Screenshot 4)
    moveServoSmooth(0, 52, 106, steps, delayTime);
    moveServoSmooth(1, 125, 80, steps, delayTime);
    moveServoSmooth(2, 34, 20, steps, delayTime);
    moveServoSmooth(3, 140, 157, steps, delayTime);
    moveServoSmooth(15, 140, 102, steps, delayTime);
    moveServoSmooth(4, 100, 55, steps, delayTime);
    moveServoSmooth(5, 78, 97, steps, delayTime);
    moveServoSmooth(6, 78, 179, steps, delayTime);
    moveServoSmooth(7, 173, 165, steps, delayTime);
    moveServoSmooth(14, 165, 173, steps, delayTime);
    delay(500);

    // Move back to POSE 1
    moveServoSmooth(0, 106, 45, steps, delayTime);
    moveServoSmooth(1, 80, 126, steps, delayTime);
    moveServoSmooth(2, 20, 31, steps, delayTime);
    moveServoSmooth(3, 157, 160, steps, delayTime);
    moveServoSmooth(15, 102, 103, steps, delayTime);
    moveServoSmooth(4, 55, 120, steps, delayTime);
    moveServoSmooth(5, 97, 42, steps, delayTime);
    moveServoSmooth(6, 179, 99, steps, delayTime);
    moveServoSmooth(7, 165, 170, steps, delayTime);
    moveServoSmooth(14, 173, 174, steps, delayTime);
    delay(500);

    server.send(200, "text/plain", "Show Demo completed");
}

void setup() {
    Serial.begin(9600);
    Serial.println("Starting...");

    // Initialize PCA9685
    pca.begin();
    pca.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Define web server routes
    server.on("/", handleRoot);
    server.on("/moveServo", handleMoveServo);
    server.on("/getAngles", handleGetAngles);
    server.on("/calibrate", handleCalibrate);
    server.on("/demo", handleDemo);
    server.on("/printAngles", handlePrintAngles);
    server.on("/showDemo", handleShowDemo);

    // Start the server
    server.begin();
    Serial.println("Server started");
}

void loop() {
    // Handle client requests
    server.handleClient();

    if (autoMode) {
        // Add autonomous behavior here
        // Example: Move servos in a predefined sequence
    }
}
