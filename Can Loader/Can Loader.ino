/*
 Name:		Can_Loader.ino
 Created:	1/14/2026 9:19:23 AM
 Author:	grayl
*/

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_MotorShield.h>

// =============================================================
// HARDWARE CONFIGURATION
// =============================================================

// Pin Assignments
constexpr int RED_LED = 13;
constexpr int BUTTON_A = 15;
constexpr int BUTTON_B = 32;
constexpr int BUTTON_C = 14;

// Magnetic Sensor Pins (Left to Right)
const int SENSOR_PINS[5] = { 19, 21, 7, 8, 37 };
constexpr int CENTER_SENSOR_INDEX = 2; // Index of the center sensor (0-based)

// Display
Adafruit_SH1107 display(64, 128, &Wire);
char rotatingSymbols[] = { '-', '\\', '|', '/' };
int  symbolIndex = 0;

// Motor Shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1); // 200 steps/rev, Port 1

// ESP-NOW Communication
// Vehicle MAC address (get from Ce 6-8 II Serial Monitor on startup)
uint8_t vehicleMacAddress[] = {0xE8, 0x9F, 0x6D, 0x20, 0x56, 0x28}; // Update with actual MAC
esp_now_peer_info_t peerInfo;
bool espnowConnected = false;
constexpr int ESPNOW_BUF_LEN = 250;

// Configurable Speeds in mm/s
constexpr int SPEED_NORMAL = 100;
constexpr int SPEED_SLOW = 15;
constexpr int SPEED_STOP = 0;

// Timing Configuration
constexpr unsigned long CONTROL_LOOP_INTERVAL = 5;    // ms
constexpr unsigned long DISPLAY_UPDATE_INTERVAL = 100; // ms
constexpr unsigned long SENSOR_DEBOUNCE_TIME = 5;     // ms
constexpr unsigned long DOCKED_STABILIZE_TIME = 1000;  // ms
constexpr unsigned long DEPARTING_MIN_TIME = 1000;     // ms
constexpr unsigned long APPROACH_TIMEOUT = 5000;       // ms - Max time between sensors
constexpr unsigned long SENSOR_CLEARANCE_TIMEOUT = 1500; // ms - Time to wait after last sensor to confirm departure
constexpr unsigned long STATE_TIMEOUT = 30000;         // 30s safety timeout
constexpr unsigned long COMMAND_RESEND_INTERVAL = 500; // ms

// Direction Control
enum DirectionMode { DIR_BOTH, DIR_L_TO_R, DIR_R_TO_L };
DirectionMode currentDirectionMode = DIR_BOTH;
int commandPolarity = 1;      // 1 = Normal, -1 = Inverted Speed Command
int approachSign = 1;         // 1 = L->R, -1 = R->L
bool lastButtonBState = HIGH; // For debouncing/edge detection

// State Machine
enum LoaderState { 
 IDLE, 
 IGNORING,
 APPROACHING, 
 DOCKED, 
 CORRECTING,
 LOADING, 
 DEPARTING 
};

constexpr int STATUS_MSG_COUNT = 3;
String statusMessages[STATUS_MSG_COUNT] = { "Waiting", "", "" };
void addStatusMessage(const String& msg) {
	for (int i = STATUS_MSG_COUNT - 1; i > 0; i--) {
		statusMessages[i] = statusMessages[i - 1];
	}
	statusMessages[0] = msg;
}

LoaderState currentState = LoaderState::IDLE;
bool shieldFound = false;
unsigned long stateTimer = 0;
unsigned long lastControlLoop = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastCommandSent = 0;
unsigned long lastSensorChange = 0;
unsigned long lastSensorActive = 0;
bool prevAnySensor = false;
float vehiclePosition = -1.0; // 0.0=FarLeft, 3.0=Center, 6.0=FarRight, -1=Unknown
int travelDirection = 0;      // -1=Left, 1=Right, 0=Unknown

// Loading Sequence State
enum LoadingStep { LOAD_IDLE, LOAD_EXTEND, LOAD_WAIT, LOAD_RETRACT, LOAD_DONE };
LoadingStep loadingStep = LOAD_IDLE;
unsigned long loadingStepTimer = 0;
int stepsRemaining = 0;

// Function Prototypes
void updateDisplay();
void sendSpeedCommand(int speed);
void sendAccelCommand(int accel);
void handleSensors();
void runLoadingSequence();
int getSensorPattern(bool sensors[]);
void printSensorDiagnostics(bool sensors[]);
void onEspNowDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status);
void addStatusMessage(const String& msg);

void setup()
{
	Serial.begin(115200);
	Serial.println("Starting Can Loader...");

	// Initialize display
	display.begin(0x3C, true);
	display.setRotation(1);
	display.clearDisplay();
	display.display();
	display.setTextSize(1);
	display.setTextColor(SH110X_WHITE);
	display.setCursor(0, 0);
	display.println(F("Initializing..."));
	display.display();

	// Configure button inputs with pullups
	pinMode(BUTTON_A, INPUT_PULLUP);
	pinMode(BUTTON_B, INPUT_PULLUP);
	pinMode(BUTTON_C, INPUT_PULLUP);

    // Configure Sensor inputs
    for (int i = 0; i < 5; i++) {
        pinMode(SENSOR_PINS[i], INPUT);
    }

	// Configure LED outputs
	pinMode(RED_LED, OUTPUT);
	digitalWrite(RED_LED, LOW);

	// Initialize Motor Shield
	if (!AFMS.begin()) {
		Serial.println("Could not find Motor Shield. Check wiring.");
        addStatusMessage("No Shield");
		shieldFound = false;
	} else {
		shieldFound = true;
		myMotor->setSpeed(60); // 60 RPM
		Serial.println("Motor Shield found.");
	}

	// Initialize WiFi in Station mode (required for ESP-NOW)
	WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    delay(100);
	WiFi.disconnect();
	Serial.print("Can Loader MAC Address: ");
	Serial.println(WiFi.macAddress());

	// Initialize ESP-NOW
	if (esp_now_init() != ESP_OK) {
		Serial.println("ESP-NOW Init Failed!");
		addStatusMessage("ESP-NOW Fail");
	} else {
		Serial.println("ESP-NOW Init Success");
		
		// Register send callback
		esp_now_register_send_cb(onEspNowDataSent);
		
		// Register peer (vehicle)
		memcpy(peerInfo.peer_addr, vehicleMacAddress, 6);
		peerInfo.channel = 0; // Use current channel
		peerInfo.encrypt = false;
		
		if (esp_now_add_peer(&peerInfo) != ESP_OK) {
			Serial.println("Failed to add peer");
			addStatusMessage("Peer Add Fail");
		} else {
			Serial.println("Peer added successfully");
			addStatusMessage("ESP-NOW Ready");
			espnowConnected = true;
		}
	}

	updateDisplay();
	Serial.println("Setup Finished");
}

void loop()
{
    // Run control loop at fixed interval (5ms)
    if (millis() - lastControlLoop < CONTROL_LOOP_INTERVAL) {
        return; // Not time yet
    }
    lastControlLoop = millis();

    // Handle State Machine
    handleSensors();

    // Manual Override check (Jogging)
	if (shieldFound && currentState == IDLE) { // Only jog when idle
        bool btnA = !digitalRead(BUTTON_A);
        bool btnC = !digitalRead(BUTTON_C);
        
        if (btnA) {
            myMotor->step(1, FORWARD, DOUBLE);
            addStatusMessage("Jog CW");
        } else if (btnC) {
            myMotor->step(1, BACKWARD, DOUBLE);
            addStatusMessage("Jog CCW");
        } else {
             // Keep status as "Waiting" if not jogging
             if (statusMessages[0].startsWith("Jog")) {
                myMotor->release();
                addStatusMessage("Waiting");
             }
        }
    }
    
    // Handle Display Update
    updateDisplay();

    // Handle Button B (Direction Mode Toggle + Polarity Long Press)
    bool btnB = digitalRead(BUTTON_B);
    static unsigned long btnBPressTime = 0;
    static bool btnBPressed = false;
    static bool longPressHandled = false;

    if (lastButtonBState == HIGH && btnB == LOW) {
        // Press Start
        btnBPressTime = millis();
        btnBPressed = true;
        longPressHandled = false;
    } else if (lastButtonBState == LOW && btnB == HIGH) {
        // Release
        if (btnBPressed && !longPressHandled) {
             // Short Press Action
            if (currentDirectionMode == DIR_BOTH) currentDirectionMode = DIR_L_TO_R;
            else if (currentDirectionMode == DIR_L_TO_R) currentDirectionMode = DIR_R_TO_L;
            else currentDirectionMode = DIR_BOTH;
            
            // Show change on status instantly
            String dirStr = "Dir: ";
            if (currentDirectionMode == DIR_BOTH) dirStr += "<->";
            else if (currentDirectionMode == DIR_L_TO_R) dirStr += "L->R";
            else dirStr += "R<-L";
            addStatusMessage(dirStr);
        }
        btnBPressed = false;
    }

    // Check Long Press
    if (btnBPressed && !longPressHandled && (millis() - btnBPressTime > 1000)) {
        longPressHandled = true;
        // Toggle Polarity
        commandPolarity = -commandPolarity;
        addStatusMessage(commandPolarity == 1 ? "Pol: Right" : "Pol: Left");
    }

    lastButtonBState = btnB;
}

// ESP-NOW send callback
void onEspNowDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
	if (status == ESP_NOW_SEND_SUCCESS) {
		// Successfully sent
	} else {
		Serial.println("ESP-NOW send failed");
	}
}

void sendSpeedCommand(int speed) {
	if (!espnowConnected) {
		// Not connected, skip sending
		Serial.println("ESP-NOW not connected");
		return;
	}
	
	char message[ESPNOW_BUF_LEN];
	
	// Send command in format: "S <speed>" or "STOP"
	if (speed == 0) {
		snprintf(message, ESPNOW_BUF_LEN, "STOP");
	} else {
		snprintf(message, ESPNOW_BUF_LEN, "S %d", speed * commandPolarity);
	}
	
	// Send via ESP-NOW
	esp_err_t result = esp_now_send(vehicleMacAddress, (uint8_t *)message, strlen(message) + 1);
	
	if (result == ESP_OK) {
		Serial.print("ESP-NOW TX: ");
		Serial.println(message);
	} else {
		Serial.print("ESP-NOW send error: ");
		Serial.println(result);
	}
}

void sendAccelCommand(int accel) {
	if (!espnowConnected) return;
	
	char message[ESPNOW_BUF_LEN];
	snprintf(message, ESPNOW_BUF_LEN, "A %d", accel);
	
	esp_err_t result = esp_now_send(vehicleMacAddress, (uint8_t *)message, strlen(message) + 1);
	
	if (result == ESP_OK) {
		Serial.print("ESP-NOW TX Accel: ");
		Serial.println(message);
	}
}

void handleSensors() {
    bool sensors[5];
    bool anySensor = false;
    bool centerSensor = false;

    // Read all sensors
    for (int i = 0; i < 5; i++) {
        sensors[i] = (digitalRead(SENSOR_PINS[i]) == LOW);
        if (sensors[i]) anySensor = true;
    }
    centerSensor = sensors[CENTER_SENSOR_INDEX]; // Pin 19

    if (anySensor) {
        lastSensorActive = millis();
    }

    // Capture sensor changes (software debounce removed per hardware spec)
    if (anySensor != prevAnySensor) {
        lastSensorChange = millis();
        prevAnySensor = anySensor;
    }

    // Log sensor pattern for diagnostics (optional, only when changing)
    if (anySensor && (millis() - lastSensorChange < 10)) {
        printSensorDiagnostics(sensors);
    }

    // --- Vehicle Position Logic (Centroid) ---
    float sensorSum = 0;
    int sensorCount = 0;
    // Map sensors 0..4 to Positions 1.0, 2.0, 3.0, 4.0, 5.0
    for(int i=0; i<5; i++) {
        if(sensors[i]) {
           sensorSum += (i + 1.0); 
           sensorCount++;
        }
    }

    if (sensorCount > 0) {
        float newPos = sensorSum / sensorCount;
        // Determine direction if we moved significantly
        if (vehiclePosition > 0 && abs(newPos - vehiclePosition) > 0.1) {
            travelDirection = (newPos > vehiclePosition) ? 1 : -1;
        }
        vehiclePosition = newPos;
    } else {
         // No sensors active: Drift/Interpolate based on state for visibility
         if (currentState == CORRECTING && vehiclePosition > 0) {
             // Simulate movement towards center (3.0) at slow drift
             if (vehiclePosition > 3.0) vehiclePosition -= 0.002;
             else if (vehiclePosition < 3.0) vehiclePosition += 0.002;
         }
         else if (currentState != IDLE && vehiclePosition > 0) {
              // Normal Drift loosely in direction of travel
              if (currentState != LOADING && travelDirection != 0) {
                  vehiclePosition += (travelDirection * 0.01);
                  // Clamp to reasonable bounds
                  if (vehiclePosition < 0.0) vehiclePosition = 0.0;
                  if (vehiclePosition > 6.0) vehiclePosition = 6.0;
              }
         }
    }
    // -----------------------------------------------

    // State timeout safety check
    if (currentState != IDLE && (millis() - stateTimer > STATE_TIMEOUT)) {
        Serial.println("State timeout! Resetting to IDLE");
        currentState = IDLE;
        addStatusMessage("Timeout Reset");
        digitalWrite(RED_LED, LOW);
        if (shieldFound) myMotor->release();
        sendAccelCommand(50); // Reset acceleration
        delay(20);
        sendSpeedCommand(SPEED_NORMAL * approachSign); // Resume normal operation
        return;
    }

    // State Machine
    switch (currentState) {
        case IDLE:
            if (anySensor) {
                // Determine direction based on which side triggered
                // Left side: sensors 0,1. Right side: sensors 3,4.
                bool fromLeft = sensors[0] || sensors[1];
                bool fromRight = sensors[3] || sensors[4];
                bool validDirection = false;

                if (currentDirectionMode == DIR_BOTH) {
                    validDirection = true;
                } else if (currentDirectionMode == DIR_L_TO_R) {
                    // Valid if coming from Left (so left sensors hit first)
                    // If conflicting (both L and R hit same time), assume valid to be safe or invalid? 
                    // Let's assume valid if ONLY left is hit, or if Left is hit and Right isn't.
                    if (fromLeft && !fromRight) validDirection = true;
                    // If Just Center hit (weird), we might need to wait or assume invalid. Assume invalid for strictness.
                } else if (currentDirectionMode == DIR_R_TO_L) {
                    if (fromRight && !fromLeft) validDirection = true;
                }

                if (validDirection) {
                    currentState = APPROACHING;
                    stateTimer = millis();
                    
                    // Determine approach direction
                    if (fromRight && !fromLeft) {
                        approachSign = -1; // Moving Right to Left
                    } else {
                        approachSign = 1;  // Moving Left to Right (default)
                    }

                    sendAccelCommand(999); // High acceleration for loader control
                    delay(20);
                    sendSpeedCommand(SPEED_SLOW * approachSign);
                    lastCommandSent = millis();
                    addStatusMessage("Approach");
                    digitalWrite(RED_LED, HIGH);
                } else {
                    currentState = IGNORING;
                    stateTimer = millis(); // Timeout safety applies here too
                    addStatusMessage("Ignoring");
                    // Do NOT send command, let it pass
                }
            }
            break;

        case IGNORING:
            // Wait until vehicle leaves all sensors (with clearance timeout)
            // Ensure no sensors have been active for SENSOR_CLEARANCE_TIMEOUT
            if ((millis() - lastSensorActive > SENSOR_CLEARANCE_TIMEOUT)) {
                currentState = IDLE;
                addStatusMessage("Waiting");
                digitalWrite(RED_LED, LOW);
            }
            break;

        case APPROACHING:
            // Resend slow command periodically for reliability
            if (millis() - lastCommandSent >= COMMAND_RESEND_INTERVAL) {
                sendSpeedCommand(SPEED_SLOW * approachSign);
                lastCommandSent = millis();
            }

            if (centerSensor) {
                currentState = DOCKED;
                sendSpeedCommand(SPEED_STOP);
                lastCommandSent = millis();
                addStatusMessage("Docked");
                stateTimer = millis();
            } else if (!anySensor && (millis() - lastSensorActive > APPROACH_TIMEOUT)) {
                // Vehicle passed without stopping - missed detection
                Serial.println("Vehicle passed without docking");
                currentState = IDLE;
                addStatusMessage("Missed");
                digitalWrite(RED_LED, LOW);
                sendAccelCommand(50); // Reset acceleration
                delay(20);
                sendSpeedCommand(SPEED_NORMAL * approachSign);
                delay(1000); // Show message briefly
                addStatusMessage("Waiting");
            }
            break;

        case DOCKED:
            // Check for sensor loss or overshoot
            if (!centerSensor) {
                // If center lost, assume correction needed (even if in gap)
                currentState = CORRECTING;
                addStatusMessage("Correcting");
                break;
            }

            // Keep sending stop command for safety
            if (millis() - lastCommandSent >= COMMAND_RESEND_INTERVAL) {
                sendSpeedCommand(SPEED_STOP);
                lastCommandSent = millis();
            }
            
            // Wait for vehicle to stabilize
            if (millis() - stateTimer > DOCKED_STABILIZE_TIME) {
                currentState = LOADING;
                loadingStep = LOAD_IDLE; // Initialize loading sequence
                addStatusMessage("Loading");
                stateTimer = millis();
            }
            break;

        case CORRECTING:
             // Attempt to return to center sensor
             if (centerSensor) {
                 currentState = DOCKED;
                 sendSpeedCommand(SPEED_STOP);
                 lastCommandSent = millis();
                 addStatusMessage("Re-Docked");
                 stateTimer = millis(); // Reset stability timer
             } else {
                 // Check if truly lost (timeout)
                 if (!anySensor && (millis() - lastSensorActive > SENSOR_CLEARANCE_TIMEOUT)) {
                     currentState = IDLE;
                     addStatusMessage("Lost");
                 } else {
                     // Drive towards center (Position 3.0)
                     if (millis() - lastCommandSent >= COMMAND_RESEND_INTERVAL) {
                         // If Pos > 3.0 (Right), move Left (-).
                         // If Pos < 3.0 (Left), move Right (+).
                         // Use estimated vehiclePosition which drifts in gaps
                         int correctSpeed = (vehiclePosition > 3.0) ? -SPEED_SLOW : SPEED_SLOW;
                         sendSpeedCommand(correctSpeed);
                         lastCommandSent = millis();
                     }
                 }
             }
             break;

        case LOADING:
            // Keep vehicle stopped during loading
            if (millis() - lastCommandSent >= COMMAND_RESEND_INTERVAL) {
                sendSpeedCommand(SPEED_STOP);
                lastCommandSent = millis();
            }
            
            // Run non-blocking loading sequence
            runLoadingSequence();
            
            // Check if loading is complete
            if (loadingStep == LOAD_DONE) {
                currentState = DEPARTING;
                stateTimer = millis();
                sendSpeedCommand(SPEED_NORMAL * approachSign);
                lastCommandSent = millis();
                addStatusMessage("Departing");
            }
            break;

        case DEPARTING:
            // Resend normal speed command
            if (millis() - lastCommandSent >= COMMAND_RESEND_INTERVAL) {
                sendAccelCommand(50); // Reset acceleration
                sendSpeedCommand(SPEED_NORMAL * approachSign);
                lastCommandSent = millis();
            }
            
            // Wait until vehicle clears all sensors (with clearance timeout)
            // Must satisfy MIN_TIME AND have no sensors active for CLEARANCE_TIMEOUT
            if ((millis() - stateTimer > DEPARTING_MIN_TIME) && 
                (millis() - lastSensorActive > SENSOR_CLEARANCE_TIMEOUT)) {
                currentState = IDLE;
                addStatusMessage("Waiting");
                digitalWrite(RED_LED, LOW);
            }
            break;
    }
}

void runLoadingSequence() {
    if (!shieldFound) {
        loadingStep = LOAD_DONE;
        return;
    }
    
    switch (loadingStep) {
        case LOAD_IDLE:
            // Start loading sequence
            loadingStep = LOAD_EXTEND;
            stepsRemaining = 400; // 2 rotations CW
            addStatusMessage("Extending");
            break;
            
        case LOAD_EXTEND:
            // Step motor incrementally (non-blocking)
            if (stepsRemaining > 0) {
                int steps = min(stepsRemaining, 200); // Move 10 steps at a time
                myMotor->step(steps, FORWARD, DOUBLE);
                stepsRemaining -= steps;
            } else {
                loadingStep = LOAD_WAIT;
                loadingStepTimer = millis();
                addStatusMessage("Can Loaded");
            }
            break;
            
        case LOAD_WAIT:
            // Wait for 500ms
            if (millis() - loadingStepTimer >= 500) {
                loadingStep = LOAD_RETRACT;
                stepsRemaining = 400; // 2 rotations CCW
                addStatusMessage("Retracting");
            }
            break;
            
        case LOAD_RETRACT:
            // Step motor back incrementally
            if (stepsRemaining > 0) {
                int steps = min(stepsRemaining, 200);
                myMotor->step(steps, BACKWARD, DOUBLE);
                stepsRemaining -= steps;
            } else {
                myMotor->release();
                loadingStep = LOAD_DONE;
                addStatusMessage("Load Done");
            }
            break;
            
        case LOAD_DONE:
            // Sequence complete - handled by state machine
            break;
    }
}

void updateDisplay()
{
    // Only update display at fixed interval
    if (millis() - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL) {
        return;
    }
    lastDisplayUpdate = millis();

	display.clearDisplay();

    // Top Left: Mode + Polarity String
    display.setTextSize(1);
    display.setCursor(0, 0);
    
    String modeStr = "";
    if (commandPolarity == -1) modeStr += "+";
    
    // Construct base mode string
    if (currentDirectionMode == DIR_BOTH) modeStr += "L<->R";
    else if (currentDirectionMode == DIR_L_TO_R) modeStr += "L->R";
    else modeStr += "L<-R";
    
    if (commandPolarity == 1) modeStr += "+";
    
    display.print(modeStr);

    // Top Right: ESP-NOW Status Symbol (Blinking when active)
    int espX = 120;
    int espY = 3;
    int espR = 2;
    
    // Check if we are actively sending commands (within last 250ms)
    bool isTransmitting = (millis() - lastCommandSent < 250);
    // Blink every 250ms cycle
    bool blinkOn = (millis() % 250) < 125;
    
    if (espnowConnected) {
        if (isTransmitting) {
             if (blinkOn) display.fillCircle(espX, espY, espR, SH110X_WHITE);
             else display.drawCircle(espX, espY, espR, SH110X_WHITE);
        } else {
             display.fillCircle(espX, espY, espR, SH110X_WHITE);
        }
    } else {
         display.drawCircle(espX, espY, espR, SH110X_WHITE);
    }

    // Display Status Queue
    // Move Rotating Symbol to left of messages at (0, 10)
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.print(rotatingSymbols[symbolIndex]);
    symbolIndex = (symbolIndex + 1) % 4;

    // Messages indented to x=10
    for (int i = 0; i < STATUS_MSG_COUNT; i++) {
        display.setCursor(10, 10 + i * 10);
        display.print(statusMessages[i]);
    }

    // Draw Track and Vehicle
    int trackY = 60;
    int trackWidth = 120;
    int trackX = 4;
    // Track Line
    display.drawLine(trackX, trackY, trackX + trackWidth, trackY, SH110X_WHITE);
    // Sensor ticks (1.0 to 5.0 -> mapped to track)
    // Scale: 0.0=Left(X), 6.0=Right(X+W)
    for(int i=1; i<=5; i++) {
       int sx = trackX + (i * (trackWidth / 6));
       display.drawFastVLine(sx, trackY - 2, 5, SH110X_WHITE);
    }
    // Vehicle
    if (vehiclePosition >= 0) {
        int vx = trackX + (int)(vehiclePosition * (trackWidth / 6.0));
        // Draw centered box
        display.fillRect(vx - 3, trackY - 4, 7, 5, SH110X_WHITE);
    }
    
	display.display();
}

// Helper function: Get sensor pattern count (how many sensors active)
int getSensorPattern(bool sensors[]) {
    int count = 0;
    for (int i = 0; i < 5; i++) {
        if (sensors[i]) count++;
    }
    return count;
}

// Helper function: Print sensor diagnostics to Serial
void printSensorDiagnostics(bool sensors[]) {
    Serial.print("Sensors: [");
    for (int i = 0; i < 5; i++) {
        Serial.print(sensors[i] ? "1" : "0");
        if (i < 4) Serial.print(",");
    }
    Serial.print("] Center: ");
    Serial.print(sensors[CENTER_SENSOR_INDEX] ? "YES" : "NO");
    Serial.print(" Count: ");
    Serial.println(getSensorPattern(sensors));
}
