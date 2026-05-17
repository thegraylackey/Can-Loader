/*
 ===============================================================
  Can_Loader.ino
 ===============================================================
  Created : 1/14/2026
  Author  : grayl
  Repo    : https://github.com/thegraylackey/Can-Loader
  Branch  : master

 ===============================================================
  SYSTEM OVERVIEW
 ===============================================================
  O-scale model railroad automatic can loader. A stationary
  loader station intercepts passing wagons, detects whether
  they are already loaded via a reflective sensor, and if empty
  extends a pair of arms to drop a can from an overhead hopper
  into the wagon bay before releasing the wagon.

  The wagon (vehicle) is a self-propelled ESP-NOW receiver.
  The loader (this device) is the ESP-NOW transmitter.
  Speed/accel commands are sent as ASCII strings:
	"S <signed_mm_per_s>"  — set speed (polarity-aware)
	"STOP"                 — zero speed
	"A <mm_per_s_sq>"      — set acceleration

 ===============================================================
  CONTROLLER HARDWARE
 ===============================================================
  Board   : Adafruit ESP32 Feather V2
  CPU     : ESP32-S3 (or ESP32 depending on Feather V2 variant)
  Flash   : 4 MB
  WiFi    : 802.11 b/g/n — used exclusively for ESP-NOW (ch 1)
  Display : Adafruit SH1107 128×64 OLED, I2C @ 0x3C, rotation 1
			(Adafruit_SH110X / Adafruit_GFX libraries)
  Steppers: 2× TMC2209 V1.3 stepper driver boards
			STEP/DIR/EN interface (UART not used)
			Microstepping: 1/8 (MS1=GND, MS2=GND default)
			Stepper motors: 200 step/rev NEMA 17 (or equiv.)

 ===============================================================
  PIN ASSIGNMENTS  (ESP32 Feather V2)
 ===============================================================
  PIN      GPIO  FUNCTION
  -------  ----  -----------------------------------------------
  A0        26   Motor 1 STEP       (DAC2, ADC2)
  A1        25   Motor 1 DIR        (DAC1, ADC2)
  A2        34   Home Switch 1      (INPUT ONLY, ADC1) *
  A3        39   Can Sensor         (INPUT ONLY, ADC1) *
  A4        36   Home Switch 2      (INPUT ONLY, ADC1) *
  A5         4   Motor 2 EN         (ADC2)
  SCK        5   (Available)
  MO        19   Wagon Sensor 0     (leftmost)
  MI        21   Wagon Sensor 1
  RX         7   Wagon Sensor 2     (CENTER — index 2)
  TX         8   Wagon Sensor 3
  D37       37   Wagon Sensor 4     (rightmost, INPUT ONLY, ADC1) *
  D13       13   Red Status LED     (LEDC PWM ch, ADC2)
  D12       12   Motor 2 DIR        (ADC2, BOOT PIN — must be LOW at boot)
  D27       27   Motor 2 STEP       (ADC2)
  D33       33   Motor 1 EN         (ADC1)
  D15       15   Button A           (INPUT_PULLUP, ADC2)
  D32       32   Button B           (INPUT_PULLUP, ADC1)
  D14       14   Button C           (INPUT_PULLUP, ADC2)
  SCL       20   I2C Clock          (OLED display)
  SDA       22   I2C Data           (OLED display)

  * INPUT_ONLY pins: no internal pull-up available.
	Home switches (34, 36) require external pull-up resistors.
	Can sensor (39) and sensor 4 (37) are passive inputs.

 ===============================================================
  SENSOR DETAILS
 ===============================================================
  Wagon Position Sensors (5×):
	Hall-effect or reed-switch sensors mounted along the track.
	Active LOW (sensor fires → GPIO reads LOW).
	Positions mapped 1.0–5.0 (left to right); centroid of active
	sensors gives estimated wagon position and travel direction.
	Center sensor (index 2, GPIO 7) is the dock reference.

  Home Switches (2×):
	Magnetic limit switches, Active LOW.
	Fire when the loader arms are fully retracted (home position).
	External pull-up resistors required on GPIO 34 and 36.
	Both switches must be active for the system to be considered
	homed. Checked every second in IDLE; re-homing triggered if
	either switch is lost.

  Can Sensor (GPIO 39):
	Reflective IR sensor aimed at the wagon bay beneath the arms.
	Active HIGH (can present in wagon → GPIO reads HIGH).
	Reliable alignment only when wagon is near center sensors
	(sensors 1–3). Sampled continuously during APPROACHING,
	DOCKED, and LOADING states.
	  APPROACHING/DOCKED: HIGH → wagon already loaded, skip to DEPARTING.
	  LOADING WAIT      : HIGH → can dropped successfully, retract.
						  LOW  → hopper empty, retract without load.
	Latched as wagonHasCan — once set, loading is suppressed for
	the entire wagon cycle (reset only on return to IDLE).

 ===============================================================
  STEPPER MECHANICS
 ===============================================================
  Drive chain    : Motor → 1-start worm (3 mm pitch) → 9-tooth worm wheel
				   → pinion (12T, PD 12 mm) → rack (pitch π mm)
  Worm reduction : 9 motor revs → 1 worm wheel rev (= 1 pinion rev)
  Rack travel    : 1 pinion rev = π × PD = 12π mm of linear travel
  Travel/rev     : 12π / 9 ≈ 4.19 mm per motor revolution
  Steps/rev      : 200 full steps × 8 microsteps = 1600 steps/rev
  Full stroke    : LOAD_TRAVEL_MM = 89 mm → ~33,987 microsteps
  Home position  : Both arms fully retracted, home switches LOW
  Extend target  : TOTAL_LOAD_STEPS (computed from above)
  Homing speed   : HOMING_SPEED = 2500 steps/s
  Run speed      : STEPPER_MAX_SPEED = 15000 steps/s
  Acceleration   : STEPPER_ACCEL = 10000 steps/s²
  Homing settle : HOMING_SETTLE_STEPS = -2500 steps (past switch, into hard endstop)

  Step generation: FastAccelStepper (hardware RMT/timer — no
  loop() polling required, display updates freely while moving).

  Both steppers mirror each other during normal operation via the
  paired-stepper helpers (motorsMoveTo / motorsMove / etc.).
  Homing drives each stepper independently to allow individual
  switch detection and arm-sync fault detection.

 ===============================================================
  STATE MACHINE
 ===============================================================
  Primary states (LoaderState):
	IDLE        — Waiting for a wagon. Home switches verified
				  every second. wagonHasCan reset here.
	IGNORING    — Wagon detected but direction filter rejected it,
				  or wagon was tracked leaving. Waits for sensors
				  to clear before returning to IDLE.
	APPROACHING — Wagon detected, slow approach command sent.
			  Can sensor sampled every tick. If can detected,
			  aborts immediately to DEPARTING at normal speed.
	DOCKED      — Center sensor active, wagon stopped. Stabilize
				  dwell (500 ms) then load or depart if loaded.
	CORRECTING  — Center sensor lost while docked. Sends reverse
				  correction command. Returns to DOCKED if center
				  re-acquired, or IDLE after CORRECTING_LOST_TIMEOUT.
	LOADING     — Arms extend, dwell, retract (see Loading Steps).
	DEPARTING   — Wagon released at normal speed. Minimum dwell
				  (DEPARTING_MIN_TIME) before returning to IDLE.
	HOMING      — Non-blocking homing sequence (see Homing Phases).
	ERROR       — Sticky fault state. All automation suspended.
				  Manual jog permitted. Button B clears and re-homes.

  Loading steps (LoadingStep):
	IDLE    → EXTEND  — Hard gate: abort if can already present.
	EXTEND            — Wait for both arms to reach TOTAL_LOAD_STEPS.
						Arm-sync and extend-timeout faults monitored.
	WAIT              — Dwell at full extension (LOAD_DWELL_TIME).
						If hopper empty, retract without load (non-fatal).
	RETRACT           — Return both arms to 0. Arm-sync fault monitored.
						Can sensor HIGH after retract = can seated in wagon (success).
	DONE              — Sequence complete; state machine advances to DEPARTING.

  Homing phases (HomingPhase):
	RETRACTING — Both arms drive toward home switches at HOMING_SPEED.
				 Individual arm completion tracked; arm-sync fault if one
				 arm lags by more than ARM_SYNC_TIMEOUT (1 s).
				 Overall HOMING_TIMEOUT (10 s) safety limit.
	SETTLING   — Both arms press HOMING_SETTLE_STEPS past the home switch and
				 seat against the hard mechanical endstop for a stable load position.
	DONE       — Position zeroed, motors disabled. WAGON_AT_STARTUP
				 checked (first boot only) via center sensor.

 ===============================================================
  ERROR CODES
 ===============================================================
  NONE               — No fault
  STATE_TIMEOUT      — A non-IDLE/HOMING/ERROR state exceeded
					   STATE_TIMEOUT (15 s) without completing
  HOMING_TIMEOUT     — Arms did not reach home switches within
					   HOMING_TIMEOUT (10 s)
  WAGON_AT_STARTUP   — Center sensor active at first boot; wagon
					   present before system was ready (checked once)
  ARM_SYNC           — One arm reached home/retract target but the
					   other did not follow within ARM_SYNC_TIMEOUT (1 s)
  EXTEND_TIMEOUT     — Arms did not reach full extension within
					   EXTEND_TIMEOUT (5 s)
  EXTEND_ARM_SYNC    — Arm sync failure during extension phase
  WAGON_NOT_DEPARTING— Center sensor still active WAGON_NOT_DEPARTING_TIME
					   (5 s) after departure commanded
  SENSOR_STUCK       — A wagon sensor has been continuously active for
					   more than SENSOR_STUCK_TIME (30 s) in IDLE
  ESPNOW_TIMEOUT     — Consecutive ESP-NOW send failures exceeded
					   ESPNOW_FAIL_TIMEOUT (10 s)

 ===============================================================
  USER INTERFACE
 ===============================================================
  Button A (GPIO 15): Jog arms DOWN (extend direction)
					  Only active in IDLE (homed) or ERROR state.
  Button B (GPIO 32): Short press — cycle direction filter (BOTH /
						L→R / R←L). In ERROR: clear fault + re-home.
					  Long press (>1 s) — toggle command polarity
						(Normal / Inverted). Use when vehicle
						direction is reversed relative to sensors.
  Button C (GPIO 14): Jog arms UP (retract direction)
					  Only active in IDLE (homed) or ERROR state.

  Red LED (GPIO 13, LEDC PWM):
	Breathing (slow sine)  — IDLE, no motion
	Slow blink (500 ms)    — Arms moving (normal operation)
	Fast blink (125 ms)    — Homing active OR error state

  OLED Display (128×64, front-view schematic):
	Top bar    — State abbreviation (left) + TX indicator (right).
				 Arm shafts run from the very top of the screen.
	Arm zone   — Two L-shaped arms (vertical shaft + inward foot)
				 descend as steppers extend. 4×2 home-switch cap at
				 the shaft top: filled = switch active (homed),
				 outline = not at home.
	Wagon zone — 16×6 px wagon outline moves with vehiclePosition;
				 filled solid when can sensor is active (can loaded),
				 outline only when empty. Gap estimate drives position
				 when sensors are clear.
	Track zone — Double horizontal rails.
	Status bar — Rotating symbol + status message (left) and
				 direction/polarity mode (right) on one line, plus
				 a second status message line below.
	Error screen (full override) — Flashing ERROR with fault name
				 and recovery instruction.

 ===============================================================
  KNOWN HARDWARE NOTES / GOTCHAS
 ===============================================================
  - GPIO 12 (Motor 2 DIR) is a boot-strapping pin. It must be
	LOW at power-on or the ESP32 will enter download mode. The
	TMC2209 DIR input is high-impedance at boot, so this is safe
	as long as no external pull-up is placed on that line.
  - GPIO 34, 36, 37, 39 are INPUT ONLY — no internal pull-ups.
	All three home switches and the can sensor on these pins need
	external resistors (typically 10 kΩ to 3.3 V).
  - ADC2 pins (4, 12, 13, 14, 15, 25, 26, 27) cannot be used for
	ADC when WiFi/ESP-NOW is active. All uses here are digital only.
  - ESP-NOW requires WiFi to be initialised in STA mode on a fixed
	channel (ch 1). WiFi.disconnect() is called after init to
	prevent association attempts.
  - TMC2209 UART is NOT used. Drivers are configured via
	hardware MS1/MS2 pins and the STEP/DIR/EN interface only.
	Default 1/8 microstepping is assumed.
  - FastAccelStepper uses ESP32 RMT or timer hardware for pulse
	generation. stepperService() in loop() is intentionally empty;
	do not add AccelStepper-style run() calls.
 ===============================================================
*/

// =============================================================
#pragma region Includes
// =============================================================
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <FastAccelStepper.h>
#pragma endregion

// =============================================================
#pragma region Hardware Configuration
// =============================================================

// ESP32 Feather V2 Pin Assignments
// =================================================
// BOTTOM ROW (Left to Right)
// =================================================
constexpr int PIN_A0_26 = 26;  // USED: Motor 1 STEP (DAC2, ADC2)
constexpr int PIN_A1_25 = 25;  // USED: Motor 1 DIR (DAC1, ADC2)
constexpr int PIN_A2_34 = 34;  // USED: Home Switch 1 (INPUT ONLY, ADC1)
constexpr int PIN_A3_39 = 39;  // USED: Can Sensor (INPUT ONLY, ADC1)
constexpr int PIN_A4_36 = 36;  // USED: Home Switch 2 (INPUT ONLY, ADC1)
constexpr int PIN_A5_4 = 4;    // USED: Motor 2 EN (ADC2)
constexpr int PIN_SCK_5 = 5;   // Available (SPI Clock, can be repurposed as GPIO)
constexpr int PIN_MO_19 = 19;  // USED: Sensor 0 (SPI MOSI)
constexpr int PIN_MI_21 = 21;  // USED: Sensor 1 (SPI MISO)
constexpr int PIN_RX_7 = 7;    // USED: Sensor 2 CENTER (UART RX)
constexpr int PIN_TX_8 = 8;    // USED: Sensor 3 (UART TX)
constexpr int PIN_D37_37 = 37; // USED: Sensor 4 (INPUT ONLY, ADC1)

// =================================================
// TOP ROW (Left to Right)
// =================================================
constexpr int PIN_D13_13 = 13; // USED: Red LED (ADC2)
constexpr int PIN_D12_12 = 12; // USED: Motor 2 DIR (ADC2, BOOT PIN - must be LOW at boot)
constexpr int PIN_D27_27 = 27; // USED: Motor 2 STEP (ADC2)
constexpr int PIN_D33_33 = 33; // USED: Motor 1 EN (ADC1)
constexpr int PIN_D15_15 = 15; // USED: Button A (ADC2, may have PWM at boot)
constexpr int PIN_D32_32 = 32; // USED: Button B (ADC1)
constexpr int PIN_D14_14 = 14; // USED: Button C (ADC2, may have PWM at boot)
constexpr int PIN_SCL_20 = 20; // USED: I2C Clock (Display)
constexpr int PIN_SDA_22 = 22; // USED: I2C Data (Display)

// =================================================
// FUNCTIONAL PIN ALIASES
// =================================================
// UI Components
constexpr int RED_LED  = PIN_D13_13;
constexpr int BUTTON_A = PIN_D15_15;
constexpr int BUTTON_B = PIN_D32_32;
constexpr int BUTTON_C = PIN_D14_14;

// Motor 1
constexpr int STEP_1 = PIN_A0_26;
constexpr int DIR_1  = PIN_A1_25;
constexpr int EN_1   = PIN_D33_33;

// Motor 2
constexpr int STEP_2 = PIN_D27_27;
constexpr int DIR_2  = PIN_D12_12;
constexpr int EN_2   = PIN_A5_4;

// Home Switch Pins (Active LOW, INPUT ONLY pins)
constexpr int HOME_SWITCH_1 = PIN_A2_34;
constexpr int HOME_SWITCH_2 = PIN_A4_36;

// Magnetic Sensor Pins
const int SENSOR_PINS[5]    = { PIN_MO_19, PIN_MI_21, PIN_RX_7, PIN_TX_8, PIN_D37_37 };
constexpr int CENTER_SENSOR_INDEX = 2;

// Can Loaded Sensor
constexpr int CAN_SENSOR = PIN_A3_39;  // Reflective sensor: HIGH = can present

// Available: PIN_SCK_5 (5)

#pragma endregion

// =============================================================
#pragma region Global Objects
// =============================================================

Adafruit_SH1107 display(64, 128, &Wire);
char rotatingSymbols[] = { '-', '\\', '|', '/' };
int  symbolIndex = 0;

FastAccelStepperEngine stepperEngine;
FastAccelStepper* stepper  = nullptr;
FastAccelStepper* stepper2 = nullptr;

// Vehicle MAC address - update with actual MAC from vehicle Serial Monitor
uint8_t vehicleMacAddress[] = { 0xE8, 0x9F, 0x6D, 0x20, 0x56, 0x28 };
esp_now_peer_info_t peerInfo;
bool espnowConnected = false;
constexpr int ESPNOW_BUF_LEN = 250;

#pragma endregion

// =============================================================
#pragma region Configuration Constants
// =============================================================

// Speeds (mm/s)
constexpr int SPEED_NORMAL = 100;
constexpr int SPEED_SLOW   = 20;
constexpr int SPEED_STOP   = 0;

// Timing
constexpr unsigned long CONTROL_LOOP_INTERVAL    = 5;     // ms
constexpr unsigned long DISPLAY_UPDATE_INTERVAL  = 125;   // ms
constexpr unsigned long DOCKED_STABILIZE_TIME    = 500;  // ms
constexpr unsigned long DEPARTING_MIN_TIME       = 2000;  // ms
constexpr unsigned long APPROACH_TIMEOUT         = 3000;  // ms - max time without sensor after approach starts
constexpr unsigned long IGNORING_CLEARANCE_TIMEOUT = 1000;   // ms - short gap to exit IGNORING after vehicle passes
constexpr unsigned long CORRECTING_LOST_TIMEOUT    = 10000;  // ms - extended wait in CORRECTING before declaring Lost
constexpr unsigned long STATE_TIMEOUT            = 15000; // ms - safety reset
constexpr unsigned long COMMAND_RESEND_INTERVAL  = 1000;   // ms
constexpr unsigned long LOAD_DWELL_TIME          = 125;   // ms dwell at full extension
constexpr unsigned long ARM_SYNC_TIMEOUT         = 1000;  // ms max gap between arms completing retract/home
constexpr unsigned long EXTEND_TIMEOUT           = 5000; // ms max time for arms to reach full extension
constexpr unsigned long WAGON_NOT_DEPARTING_TIME = 5000;  // ms max time wagon stays on center sensor after DEPARTING begins
constexpr unsigned long SENSOR_STUCK_TIME        = 30000; // ms a sensor active this long in IDLE is assumed stuck
constexpr unsigned long ESPNOW_FAIL_TIMEOUT      = 10000; // ms consecutive send failures before raising error

// Vehicle Control
constexpr int   ACCEL_APPROACH            = 999;   // High accel for loader-controlled stop
constexpr int   ACCEL_NORMAL              = 200;    // Normal accel after vehicle release
constexpr float CENTER_POSITION_THRESHOLD = 3.0f;  // Centroid value at center sensor

// Status LED
constexpr unsigned long LED_BREATH_INTERVAL       = 2000; // ms
constexpr unsigned long LED_BLINK_NORMAL_INTERVAL = 500;  // ms
constexpr unsigned long LED_BLINK_FAST_INTERVAL   = 125;  // ms

// Stepper Mechanics
// Drive chain: 1-start worm (3mm pitch) → 9T worm wheel → 12T pinion (PD 12mm) → rack (pitch π)
// 9 motor revs = 1 pinion rev = 12π mm rack travel → 12π/9 mm per motor rev
constexpr int   STEPPER_STEPS_PER_REV = 200;
constexpr int   STEPPER_MICROSTEPS    = 8;                        // TMC2209 default 1/8 microstepping
constexpr float LINEAR_TRAVEL_PER_REV = (12.0f * PI) / 9.0f;     // ≈ 4.19 mm per motor revolution
constexpr int   LOAD_TRAVEL_MM        = 89;
constexpr long  TOTAL_LOAD_STEPS      = (long)(LOAD_TRAVEL_MM * STEPPER_STEPS_PER_REV * STEPPER_MICROSTEPS / LINEAR_TRAVEL_PER_REV);

// Stepper Motion
constexpr float        STEPPER_MAX_SPEED    = 15000.0f;
constexpr float        STEPPER_ACCEL        = 10000.0f;
constexpr float        HOMING_SPEED         = 2500.0f;
constexpr int          HOMING_SETTLE_STEPS  = -2500; // steps past home switch to hard endstop
constexpr unsigned long HOMING_TIMEOUT      = 10000; // ms

#pragma endregion

// =============================================================
#pragma region Type Definitions
// =============================================================

enum class DirectionMode { BOTH, L_TO_R, R_TO_L };

enum class LoaderState {
	IDLE,
	IGNORING,
	APPROACHING,
	DOCKED,
	CORRECTING,
	LOADING,
	DEPARTING,
	HOMING,
	ERROR
};

enum class LoadingStep { IDLE, EXTEND, WAIT, RETRACT, DONE };

enum class HomingPhase { RETRACTING, SETTLING, DONE };

enum class ErrorCode { NONE, STATE_TIMEOUT, HOMING_TIMEOUT, WAGON_AT_STARTUP, ARM_SYNC,
					  EXTEND_TIMEOUT, EXTEND_ARM_SYNC,
					  WAGON_NOT_DEPARTING, SENSOR_STUCK, ESPNOW_TIMEOUT };

constexpr int STATUS_MSG_COUNT = 3;
String statusMessages[STATUS_MSG_COUNT] = { "Waiting", "", "" };
void addStatusMessage(const String& msg) {
	for (int i = STATUS_MSG_COUNT - 1; i > 0; i--) {
		statusMessages[i] = statusMessages[i - 1];
	}
	statusMessages[0] = msg;
}

#pragma endregion

// =============================================================
#pragma region State Variables
// =============================================================

// Direction / Polarity
DirectionMode currentDirectionMode = DirectionMode::BOTH;
int  commandPolarity  = 1;    // 1 = Normal, -1 = Inverted
int  wagonApproachDir = 1;    // 1 = L->R, -1 = R->L
bool lastButtonBState = HIGH;

// State Machine
LoaderState   currentState = LoaderState::IDLE;
LoaderState   prevState    = LoaderState::IDLE;
unsigned long stateTimer   = 0;

// Timing
unsigned long lastControlLoop   = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastCommandSent   = 0;
unsigned long lastSensorChange  = 0;
unsigned long lastSensorActive  = 0;
unsigned long lastHomeSwitchLog = 0;

// Vehicle Tracking
bool  prevAnySensor     = false;
float vehiclePosition   = -1.0f; // 1.0–5.0 mapped to sensor positions, -1 = unknown
int   travelDirection   = 0;     // -1=Left, 1=Right, 0=Unknown — used by CORRECTING state
int   estimatedGapIndex = -1;    // 0–3 = gap between sensor[i] and sensor[i+1], -1 = unknown

// Loading Sequence
LoadingStep   loadingStep      = LoadingStep::IDLE;
unsigned long loadingStepTimer = 0;
unsigned long armSyncTimer     = 0;  // set when first arm completes; checked for sync timeout
unsigned long extendStartTime  = 0;  // set when EXTEND begins; checked for extend timeout
bool          wagonHasCan      = false; // set if can sensor fires during approach or dock; suppresses loading

// ESP-NOW fault tracking
unsigned long firstEspNowFailTime  = 0;     // millis() of first consecutive send failure, 0 = no active failure run
volatile bool espNowTimeoutPending = false; // set in send callback; consumed safely in main loop

// Error
ErrorCode     currentError    = ErrorCode::NONE;

// Homing
bool          isHomed         = false;
bool          isHomingActive  = false;
bool          isFirstBoot     = true;  // cleared after first successful homing; gates WAGON_AT_STARTUP check
HomingPhase   homingPhase     = HomingPhase::RETRACTING;
bool          homingM1Done   = false;
bool          homingM2Done   = false;
unsigned long homingStartTime = 0;

#pragma endregion

// =============================================================
#pragma region Function Prototypes
// =============================================================

// Setup & Loop
void setup();
void loop();

// Motor Control
void setMotorsEnabled(bool enabled);
void motorsMoveTo(long pos);
void motorsStop();
void motorsSetCurrentPosition(long pos);
void motorsSetMaxSpeed(float s);
void motorsSetAccel(float a);
bool motorsAtTarget();
void stepperService();

// Status LED
void updateStatusLED();

// Control Loop
void runControlLoop();

// Input Handling
void handleJogging();
void handleInputs();

// ESP-NOW
void onEspNowDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status);
void sendSpeedCommand(int speed);
void sendAccelCommand(int accel);

// Sensor Reading
void readSensors(bool sensors[5], bool &anySensor, bool &centerSensor);
void updateVehiclePosition(bool sensors[5], bool anySensor);

// State Machine
void runStateMachine(bool sensors[5], bool anySensor, bool centerSensor, bool stateChanged);
void handleSensors();

// Loading Sequence
void runLoadingSequence();

// Home Switches
bool checkHomeSwitch1();
bool checkHomeSwitch2();

// Can Sensor
bool isCanLoaded();

// Homing
void startHoming();
bool runHomingStateMachine();

// Error Handling
void setError(ErrorCode code, const char* msg);

// Display
void updateDisplay();

// Diagnostics
int  countActiveSensors(bool sensors[]);
void printSensorDiagnostics(bool sensors[]);
void addStatusMessage(const String& msg);
void serialLog(const char* msg);

#pragma endregion


// =============================================================
#pragma region Setup & Loop
// =============================================================

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

	// Configure button inputs
	pinMode(BUTTON_A, INPUT_PULLUP);
	pinMode(BUTTON_B, INPUT_PULLUP);
	pinMode(BUTTON_C, INPUT_PULLUP);

	// Configure sensor inputs
	for (int i = 0; i < 5; i++) {
		pinMode(SENSOR_PINS[i], INPUT);
	}
	pinMode(CAN_SENSOR, INPUT);

	// Configure Status LED (LEDC PWM)
	ledcAttach(RED_LED, 5000, 8);

	// Configure motor enable pins (Active LOW - HIGH = disabled)
	pinMode(EN_1, OUTPUT); digitalWrite(EN_1, HIGH);
	pinMode(EN_2, OUTPUT); digitalWrite(EN_2, HIGH);

	// Configure stepper step/dir pins
	pinMode(STEP_1, OUTPUT); pinMode(DIR_1, OUTPUT);
	pinMode(STEP_2, OUTPUT); pinMode(DIR_2, OUTPUT);

	// Configure home switch pins (Active LOW)
	// NOTE: pins 34/36 are INPUT_ONLY — no internal pullup; external pullup resistors required
	pinMode(HOME_SWITCH_1, INPUT);
	pinMode(HOME_SWITCH_2, INPUT);

	// Configure FastAccelStepper
	stepperEngine.init();
	stepper  = stepperEngine.stepperConnectToPin(STEP_1);
	stepper2 = stepperEngine.stepperConnectToPin(STEP_2);
	stepper->setDirectionPin(DIR_1);
	stepper2->setDirectionPin(DIR_2);
	motorsSetMaxSpeed(STEPPER_MAX_SPEED);
	motorsSetAccel(STEPPER_ACCEL);

	// Homing runs non-blocking via the state machine
	startHoming();

	// Initialize WiFi (required for ESP-NOW)
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
		esp_now_register_send_cb(onEspNowDataSent);

		memcpy(peerInfo.peer_addr, vehicleMacAddress, 6);
		peerInfo.channel = 0;
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
	stepperService();
	runControlLoop();
	updateStatusLED();
	updateDisplay();
}

#pragma endregion

// =============================================================
#pragma region Motor Control
// =============================================================

void setMotorsEnabled(bool enabled) {
	digitalWrite(EN_1, enabled ? LOW : HIGH);
	digitalWrite(EN_2, enabled ? LOW : HIGH);
}

// Paired-stepper helpers — use these for all normal (synchronised) operation.
// motorsMoveTo / motorsStop / motorsSetCurrentPosition / etc.
// Homing splits the steppers individually and bypasses these intentionally.
void motorsMoveTo(long pos)              { stepper->moveTo(pos);                        stepper2->moveTo(pos); }
void motorsStop()                        { stepper->forceStopAndNewPosition(stepper->getCurrentPosition());  stepper2->forceStopAndNewPosition(stepper2->getCurrentPosition()); }
void motorsSetCurrentPosition(long pos)  { stepper->setCurrentPosition(pos);            stepper2->setCurrentPosition(pos); }
void motorsSetMaxSpeed(float s)          { stepper->setSpeedInHz((uint32_t)s);          stepper2->setSpeedInHz((uint32_t)s); }
void motorsSetAccel(float a)             { stepper->setAcceleration((uint32_t)a);       stepper2->setAcceleration((uint32_t)a); }
bool motorsAtTarget()                    { return !stepper->isRunning() && !stepper2->isRunning(); }

// Sets the error flag and logs it. Caller must also set currentState = LoaderState::ERROR if appropriate.
void setError(ErrorCode code, const char* msg) {
	currentError = code;
	serialLog(msg);
	addStatusMessage(msg);
}

void stepperService() {
	// FastAccelStepper runs entirely in hardware — no polling needed here.
}

#pragma endregion

// =============================================================
#pragma region Status LED
// =============================================================

void updateStatusLED() {
	static unsigned long lastBlinkTime = 0;
	static bool blinkState = false;
	bool motorsMoving = stepper->isRunning() || stepper2->isRunning();

	if (isHomingActive || currentState == LoaderState::ERROR) {
		if (millis() - lastBlinkTime >= LED_BLINK_FAST_INTERVAL) {
			blinkState = !blinkState;
			lastBlinkTime = millis();
		}
		ledcWrite(RED_LED, blinkState ? 255 : 0);
	} else if (motorsMoving) {
		if (millis() - lastBlinkTime >= LED_BLINK_NORMAL_INTERVAL) {
			blinkState = !blinkState;
			lastBlinkTime = millis();
		}
		ledcWrite(RED_LED, blinkState ? 255 : 0);
	} else {
		float phase = (millis() % LED_BREATH_INTERVAL) / (float)LED_BREATH_INTERVAL;
		int brightness = (int)((sin(phase * 2.0f * PI) + 1.0f) * 127.5f);
		ledcWrite(RED_LED, constrain(brightness, 0, 127));
	}
}

#pragma endregion

// =============================================================
#pragma region Control Loop
// =============================================================

void runControlLoop() {
	if (millis() - lastControlLoop < CONTROL_LOOP_INTERVAL) return;
	lastControlLoop = millis();

	if (espNowTimeoutPending) {
		espNowTimeoutPending = false;
		setError(ErrorCode::ESPNOW_TIMEOUT, "ESP-NOW Timeout");
		currentState = LoaderState::ERROR;
	}

	if (currentState == LoaderState::IDLE && isHomed &&
		!statusMessages[0].startsWith("Jog") &&
		(millis() - lastHomeSwitchLog >= 1000)) {
		lastHomeSwitchLog = millis();
		if (!checkHomeSwitch1() || !checkHomeSwitch2()) {
			serialLog("Home check failed in IDLE - re-homing");
			addStatusMessage("Re-Homing");
			startHoming();
		}
	}

	handleSensors();
	handleJogging();
	handleInputs();
}

#pragma endregion

// =============================================================
#pragma region Input Handling
// =============================================================

void handleJogging() {
	bool canJog = (currentState == LoaderState::IDLE && isHomed) ||
				   currentState == LoaderState::ERROR;
	if (!canJog) return;

	bool btnA = !digitalRead(BUTTON_A);
	bool btnC = !digitalRead(BUTTON_C);
	constexpr int jogSteps = 1000;

	if (btnA) {
		bool inError = (currentState == LoaderState::ERROR);
		long target = inError ? stepper->getCurrentPosition() + jogSteps
							  : constrain(stepper->getCurrentPosition() + jogSteps, 0, TOTAL_LOAD_STEPS);
		setMotorsEnabled(true);
		motorsMoveTo(target);
		if (!statusMessages[0].startsWith("Jog")) addStatusMessage("Jog Down");
	} else if (btnC) {
		bool inError = (currentState == LoaderState::ERROR);
		long target = inError ? stepper->getCurrentPosition() - jogSteps
							  : constrain(stepper->getCurrentPosition() - jogSteps, 0, TOTAL_LOAD_STEPS);
		setMotorsEnabled(true);
		motorsMoveTo(target);
		if (!statusMessages[0].startsWith("Jog")) addStatusMessage("Jog Up");
	} else if (statusMessages[0].startsWith("Jog")) {
		motorsStop();
		setMotorsEnabled(false);
		if (currentState != LoaderState::ERROR) addStatusMessage("Waiting");
	}
}

void handleInputs() {
	bool btnB = digitalRead(BUTTON_B);
	static unsigned long btnBPressTime = 0;
	static bool btnBPressed = false;
	static bool longPressHandled = false;

	if (lastButtonBState == HIGH && btnB == LOW) {
		btnBPressTime = millis();
		btnBPressed = true;
		longPressHandled = false;
	} else if (lastButtonBState == LOW && btnB == HIGH) {
		if (btnBPressed && !longPressHandled) {
			if (currentState == LoaderState::ERROR) {
				// Short press in ERROR: clear error and re-home unconditionally
				motorsStop();
				setMotorsEnabled(false);
				currentError         = ErrorCode::NONE;
				espNowTimeoutPending = false;
				firstEspNowFailTime  = 0;
				isHomed              = false;
				addStatusMessage("Recovering");
				startHoming();
			} else {
				// Short press: cycle direction mode
				if      (currentDirectionMode == DirectionMode::BOTH)  currentDirectionMode = DirectionMode::L_TO_R;
				else if (currentDirectionMode == DirectionMode::L_TO_R) currentDirectionMode = DirectionMode::R_TO_L;
				else                                                     currentDirectionMode = DirectionMode::BOTH;

				String dirStr = "Dir: ";
				if      (currentDirectionMode == DirectionMode::BOTH)  dirStr += "<->";
				else if (currentDirectionMode == DirectionMode::L_TO_R) dirStr += "L->R";
				else                                                     dirStr += "R<-L";
				addStatusMessage(dirStr);
			}
		}
		btnBPressed = false;
	}

	if (btnBPressed && !longPressHandled && (millis() - btnBPressTime > 1000)) {
		longPressHandled = true;
		commandPolarity = (commandPolarity == 1) ? -1 : 1;
		addStatusMessage(commandPolarity == 1 ? "Pol: Normal" : "Pol: Invert");
	}

	lastButtonBState = btnB;
}

#pragma endregion

// =============================================================
#pragma region ESP-NOW
// =============================================================

void onEspNowDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
	// NOTE: runs in WiFi driver context — only touch volatile flags here.
	if (status != ESP_NOW_SEND_SUCCESS) {
		if (firstEspNowFailTime == 0) firstEspNowFailTime = millis();
		if (millis() - firstEspNowFailTime >= ESPNOW_FAIL_TIMEOUT) {
			espNowTimeoutPending = true; // consumed in runControlLoop()
		}
	} else {
		firstEspNowFailTime  = 0;
		espNowTimeoutPending = false;
	}
}

void sendSpeedCommand(int speed) {
	if (!espnowConnected) {
		Serial.println("ESP-NOW not connected");
		return;
	}
	char message[ESPNOW_BUF_LEN];
	if (speed == 0) {
		snprintf(message, ESPNOW_BUF_LEN, "STOP");
	} else {
		snprintf(message, ESPNOW_BUF_LEN, "S %d", speed * commandPolarity);
	}
	esp_err_t result = esp_now_send(vehicleMacAddress, (uint8_t *)message, strlen(message) + 1);
	if (result == ESP_OK) {
		Serial.print("ESP-NOW TX: "); Serial.println(message);
	} else {
		Serial.print("ESP-NOW send error: "); Serial.println(result);
	}
}

void sendAccelCommand(int accel) {
	if (!espnowConnected) return;
	char message[ESPNOW_BUF_LEN];
	snprintf(message, ESPNOW_BUF_LEN, "A %d", accel);
	esp_err_t result = esp_now_send(vehicleMacAddress, (uint8_t *)message, strlen(message) + 1);
	if (result == ESP_OK) {
		Serial.print("ESP-NOW TX Accel: "); Serial.println(message);
	}
}

#pragma endregion

// =============================================================
#pragma region Sensor Reading
// =============================================================

void readSensors(bool sensors[5], bool &anySensor, bool &centerSensor) {
	anySensor    = false;
	centerSensor = false;
	for (int i = 0; i < 5; i++) {
		sensors[i] = (digitalRead(SENSOR_PINS[i]) == LOW);
		if (sensors[i]) anySensor = true;
	}
	centerSensor = sensors[CENTER_SENSOR_INDEX];

	if (anySensor) lastSensorActive = millis();

	if (anySensor != prevAnySensor) {
		lastSensorChange = millis();
		prevAnySensor    = anySensor;
	}

	if (anySensor && (millis() - lastSensorChange < 10)) {
		printSensorDiagnostics(sensors);
	}
}

void updateVehiclePosition(bool sensors[5], bool anySensor) {
	float sensorSum   = 0;
	int   sensorCount = 0;
	for (int i = 0; i < 5; i++) {
		if (sensors[i]) { sensorSum += (i + 1.0f); sensorCount++; }
	}
	if (sensorCount > 0) {
		float newPos = sensorSum / sensorCount;
		if (vehiclePosition > 0 && fabsf(newPos - vehiclePosition) > 0.1f) {
			travelDirection = (newPos > vehiclePosition) ? 1 : -1;
		}
		vehiclePosition   = newPos;
		estimatedGapIndex = -1; // vehicle is on a sensor — no gap estimate needed
	} else if (currentState == LoaderState::IDLE) {
		vehiclePosition   = -1.0f;
		estimatedGapIndex = -1;
	} else {
		// Sensors inactive in a non-IDLE state: estimate which gap the vehicle entered
		// based on the last known sensor position and the direction it was travelling.
		if (vehiclePosition > 0 && travelDirection != 0 && estimatedGapIndex == -1) {
			// Map last sensor (0-based) and direction to one of 6 gap zones:
			//   0 = left of sensor 0,  1-4 = between sensors,  5 = right of sensor 4
			int lastSensor = constrain((int)roundf(vehiclePosition) - 1, 0, 4);
			int gap = (travelDirection == 1) ? lastSensor + 1 : lastSensor;
			estimatedGapIndex = constrain(gap, 0, 5);
		}
	}
}

#pragma endregion

// =============================================================
#pragma region State Machine
// =============================================================

void runStateMachine(bool sensors[5], bool anySensor, bool centerSensor, bool stateChanged) {
	// Safety timeout
	if (currentState != LoaderState::IDLE &&
		currentState != LoaderState::HOMING &&
		currentState != LoaderState::ERROR &&
		(millis() - stateTimer > STATE_TIMEOUT)) {
		setError(ErrorCode::STATE_TIMEOUT, "State Timeout");
		currentState = LoaderState::ERROR;
		sendAccelCommand(ACCEL_NORMAL);
		sendSpeedCommand(SPEED_STOP);
		return;
	}

	switch (currentState) {

		case LoaderState::HOMING:
			if (runHomingStateMachine()) {
				// runHomingStateMachine sets ERROR on failure; only go IDLE on clean completion
				if (currentState != LoaderState::ERROR) {
					currentState = LoaderState::IDLE;
				}
			}
			break;

		case LoaderState::IDLE:
			if (stateChanged) wagonHasCan = false; // reset per wagon cycle
			// Sensor stuck: sensor has been continuously active since lastSensorChange
			// and that change was longer ago than SENSOR_STUCK_TIME — likely a faulty sensor
			if (anySensor && prevAnySensor &&
				(millis() - lastSensorChange > SENSOR_STUCK_TIME)) {
				setError(ErrorCode::SENSOR_STUCK, "Sensor Stuck");
				currentState = LoaderState::ERROR;
				break;
			}
			if (stateChanged && !isHomed && (checkHomeSwitch1() || checkHomeSwitch2())) {
					serialLog("Home switch active on IDLE entry - re-homing");
					addStatusMessage("Re-Homing");
					startHoming();
					break;
				}
			if (anySensor) {
				// If we already have a gap estimate the vehicle was tracked moving through
				// the array and is now exiting — ignore it rather than re-triggering approach.
				if (estimatedGapIndex != -1) {
					currentState      = LoaderState::IGNORING;
					stateTimer        = millis();
					estimatedGapIndex = -1;
					addStatusMessage("Ignoring");
					break;
				}

				bool fromLeft  = sensors[0] || sensors[1];
				bool fromRight = sensors[3] || sensors[4];
				bool validDirection = false;

				if (currentDirectionMode == DirectionMode::BOTH) {
					validDirection = true;
				} else if (currentDirectionMode == DirectionMode::L_TO_R && fromLeft && !fromRight) {
					validDirection = true;
				} else if (currentDirectionMode == DirectionMode::R_TO_L && fromRight && !fromLeft) {
					validDirection = true;
				}

				if (validDirection) {
					currentState    = LoaderState::APPROACHING;
					stateTimer      = millis();
					wagonApproachDir = (fromRight && !fromLeft) ? -1 : 1;
					sendAccelCommand(ACCEL_APPROACH);
					sendSpeedCommand(SPEED_SLOW * wagonApproachDir);
					lastCommandSent = millis();
					addStatusMessage("Approach");
				} else {
					currentState = LoaderState::IGNORING;
					stateTimer   = millis();
					addStatusMessage("Ignoring");
				}
			}
			break;

		case LoaderState::IGNORING:
			if (millis() - lastSensorActive > IGNORING_CLEARANCE_TIMEOUT) {
				currentState = LoaderState::IDLE;
				addStatusMessage("Waiting");
			}
			break;

		case LoaderState::APPROACHING: {
			// Can sensor is most reliable when the wagon is near the center (sensors 1–3).
			// Sample it every tick; latch on any positive reading and abort immediately.
			bool nearCenter = sensors[1] || sensors[2] || sensors[3];
			if (isCanLoaded()) {
				if (!wagonHasCan) serialLog(nearCenter ? "Can detected (approach, near center)" : "Can detected (approach)");
				wagonHasCan = true;
			}
			// Wagon already has a can — stop it and send it away without docking
			if (wagonHasCan) {
				serialLog("Can detected during approach — aborting to depart");
				addStatusMessage("Wagon Loaded");
				sendAccelCommand(ACCEL_NORMAL);
				sendSpeedCommand(SPEED_NORMAL * wagonApproachDir);
				lastCommandSent = millis();
				currentState    = LoaderState::DEPARTING;
				stateTimer      = millis();
				break;
			}
			if (millis() - lastCommandSent >= COMMAND_RESEND_INTERVAL) {
				sendSpeedCommand(SPEED_SLOW * wagonApproachDir);
				lastCommandSent = millis();
			}
			if (centerSensor) {
				currentState = LoaderState::DOCKED;
				sendSpeedCommand(SPEED_STOP);
				lastCommandSent = millis();
				stateTimer      = millis();
				addStatusMessage("Docked");
			} else if (!anySensor && (millis() - lastSensorActive > APPROACH_TIMEOUT)) {
				serialLog("Vehicle passed without docking");
				currentState = LoaderState::IDLE;
				addStatusMessage("Missed");
				sendAccelCommand(ACCEL_NORMAL);
				sendSpeedCommand(SPEED_NORMAL * wagonApproachDir);
			}
			break;
		}

		case LoaderState::DOCKED:
			if (isCanLoaded() && !wagonHasCan) { serialLog("Can detected (docked)"); wagonHasCan = true; }
			if (!centerSensor) {
				currentState = LoaderState::CORRECTING;
				addStatusMessage("Correcting");
				break;
			}
			if (millis() - lastCommandSent >= COMMAND_RESEND_INTERVAL) {
				sendSpeedCommand(SPEED_STOP);
				lastCommandSent = millis();
			}
			if (millis() - stateTimer > DOCKED_STABILIZE_TIME) {
				if (wagonHasCan) {
					serialLog("Wagon already has can — skipping load");
					addStatusMessage("Wagon Loaded");
					currentState = LoaderState::DEPARTING;
					stateTimer   = millis();
					sendAccelCommand(ACCEL_NORMAL);
					sendSpeedCommand(SPEED_NORMAL * wagonApproachDir);
					lastCommandSent = millis();
				} else {
					currentState = LoaderState::LOADING;
					loadingStep  = LoadingStep::IDLE;
					stateTimer   = millis();
					addStatusMessage("Loading");
				}
			}
			break;

		case LoaderState::CORRECTING:
			if (centerSensor) {
				currentState = LoaderState::DOCKED;
				sendSpeedCommand(SPEED_STOP);
				lastCommandSent = millis();
				stateTimer      = millis();
				addStatusMessage("Re-Docked");
			} else if (!anySensor && (millis() - lastSensorActive > CORRECTING_LOST_TIMEOUT)) {
				serialLog("CORRECTING -> Lost: no sensors within timeout");
				currentState = LoaderState::IDLE;
				addStatusMessage("Lost");
			} else {
				// Send correction immediately on entry, then re-send on interval
				if (stateChanged || millis() - lastCommandSent >= COMMAND_RESEND_INTERVAL) {
					int correctSpeed = (travelDirection != 0)
						? -travelDirection * SPEED_SLOW
						: (vehiclePosition > CENTER_POSITION_THRESHOLD ? -SPEED_SLOW : SPEED_SLOW);
					sendSpeedCommand(correctSpeed);
					lastCommandSent = millis();
				}
			}
			break;

		case LoaderState::LOADING:
			if (millis() - lastCommandSent >= COMMAND_RESEND_INTERVAL) {
				sendSpeedCommand(SPEED_STOP);
				lastCommandSent = millis();
			}
			runLoadingSequence();
			if (loadingStep == LoadingStep::DONE && currentState == LoaderState::LOADING) {
				currentState = LoaderState::DEPARTING;
				stateTimer   = millis();
				if (checkHomeSwitch1() && checkHomeSwitch2()) {
					sendSpeedCommand(SPEED_NORMAL * wagonApproachDir);
					lastCommandSent = millis();
				}
				addStatusMessage("Departing");
			}
			break;

		case LoaderState::DEPARTING:
			if (stateChanged) {
				sendAccelCommand(ACCEL_NORMAL); // Set accel once on entry
				if (!checkHomeSwitch1() || !checkHomeSwitch2()) {
					serialLog("Home check failed on DEPARTING entry - re-homing after depart");
					isHomed = false; // will trigger re-home when IDLE is reached
				}
			}
			// Wagon not departing: center sensor still active long after departure commanded
			if (centerSensor && (millis() - stateTimer > WAGON_NOT_DEPARTING_TIME)) {
				setError(ErrorCode::WAGON_NOT_DEPARTING, "Wagon Stuck");
				currentState = LoaderState::ERROR;
				break;
			}
			if (millis() - lastCommandSent >= COMMAND_RESEND_INTERVAL) {
				if (checkHomeSwitch1() && checkHomeSwitch2()) {
					sendSpeedCommand(SPEED_NORMAL * wagonApproachDir);
				} else {
					sendSpeedCommand(SPEED_STOP);
				}
				lastCommandSent = millis();
			}
			if (millis() - stateTimer > DEPARTING_MIN_TIME) {
				currentState = LoaderState::IDLE;
				addStatusMessage("Waiting");
			}
			break;

		case LoaderState::ERROR:
			if (stateChanged) {
				sendSpeedCommand(SPEED_STOP);
				lastCommandSent = millis();
			}
			break;
	}
}

void handleSensors() {
	bool sensors[5];
	bool anySensor    = false;
	bool centerSensor = false;

	bool stateChanged = (currentState != prevState);
	prevState = currentState;

	readSensors(sensors, anySensor, centerSensor);

	// In ERROR state all automatic functions are suspended — only manual jog is permitted
	if (currentState == LoaderState::ERROR) return;

	updateVehiclePosition(sensors, anySensor);
	runStateMachine(sensors, anySensor, centerSensor, stateChanged);
}

#pragma endregion

// =============================================================
#pragma region Loading Sequence
// =============================================================

void runLoadingSequence() {
	switch (loadingStep) {
		case LoadingStep::IDLE:
			// Final hard gate: re-check can sensor immediately before extending arms
			if (wagonHasCan || isCanLoaded()) {
				serialLog("Can present at load start — aborting");
				addStatusMessage("Wagon Loaded");
				wagonHasCan  = true;
				loadingStep  = LoadingStep::DONE;
				currentState = LoaderState::DEPARTING;
				stateTimer   = millis();
				sendAccelCommand(ACCEL_NORMAL);
				sendSpeedCommand(SPEED_NORMAL * wagonApproachDir);
				lastCommandSent = millis();
				break;
			}
			loadingStep      = LoadingStep::EXTEND;
			extendStartTime  = millis();
			armSyncTimer     = 0;
			setMotorsEnabled(true);
			motorsMoveTo(TOTAL_LOAD_STEPS);
			addStatusMessage("Extending");
			break;

		case LoadingStep::EXTEND: {
			bool m1Done = !stepper->isRunning();
			bool m2Done = !stepper2->isRunning();
			// Extend arm sync: one arm reached target, other hasn't within timeout
			if ((m1Done || m2Done) && !(m1Done && m2Done) && armSyncTimer == 0) {
				armSyncTimer = millis();
			}
			if (armSyncTimer > 0 && !(m1Done && m2Done) &&
				(millis() - armSyncTimer > ARM_SYNC_TIMEOUT)) {
				motorsStop();
				setMotorsEnabled(false);
				setError(ErrorCode::EXTEND_ARM_SYNC, "Arm Stuck Extend");
				loadingStep  = LoadingStep::DONE;
				currentState = LoaderState::ERROR;
				armSyncTimer = 0;
				break;
			}
			// Extend timeout: arms haven't reached target within allowed time
			if (millis() - extendStartTime > EXTEND_TIMEOUT) {
				motorsStop();
				setMotorsEnabled(false);
				setError(ErrorCode::EXTEND_TIMEOUT, "Extend Timeout");
				loadingStep  = LoadingStep::DONE;
				currentState = LoaderState::ERROR;
				break;
			}
			if (m1Done && m2Done) {
				armSyncTimer     = 0;
				loadingStep      = LoadingStep::WAIT;
				loadingStepTimer = millis();
				addStatusMessage("Extended");
			}
			break;
		}

		case LoadingStep::WAIT:
			if (!isCanLoaded()) {
				serialLog("Chute empty — retracting without load");
				addStatusMessage("Chute Empty");
				loadingStep = LoadingStep::RETRACT;
				motorsMoveTo(0);
				break;
			}
			if (millis() - loadingStepTimer >= LOAD_DWELL_TIME) {
				loadingStep = LoadingStep::RETRACT;
				motorsMoveTo(0);
				addStatusMessage("Retracting");
			}
			break;

		case LoadingStep::RETRACT: {
			bool m1Done = !stepper->isRunning();
			bool m2Done = !stepper2->isRunning();
			// Start sync timer when first arm reaches home
			if ((m1Done || m2Done) && !(m1Done && m2Done) && armSyncTimer == 0) {
				armSyncTimer = millis();
			}
			// Error if second arm hasn't caught up within timeout
			if (armSyncTimer > 0 && !(m1Done && m2Done) &&
				(millis() - armSyncTimer > ARM_SYNC_TIMEOUT)) {
				motorsStop();
				setMotorsEnabled(false);
				setError(ErrorCode::ARM_SYNC, "Arm Stuck Retract");
				loadingStep  = LoadingStep::DONE; // abort sequence
				currentState = LoaderState::ERROR;
				armSyncTimer = 0;
				break;
			}
			if (m1Done && m2Done) {
				armSyncTimer = 0;
				setMotorsEnabled(false);
				loadingStep = LoadingStep::DONE;
				if (currentError != ErrorCode::NONE) {
					// Error already set — just park, don't overwrite
					currentState = LoaderState::ERROR;
				} else {
					// Can sensor HIGH after retract = can seated in wagon — success
					addStatusMessage("Load Done");
				}
			}
			break;
		}

		case LoadingStep::DONE:
			break;
	}
}

#pragma endregion

// =============================================================
#pragma region Home Switches
// =============================================================

bool checkHomeSwitch1() { return (digitalRead(HOME_SWITCH_1) == LOW); }
bool checkHomeSwitch2() { return (digitalRead(HOME_SWITCH_2) == LOW); }
bool isCanLoaded()      { return (digitalRead(CAN_SENSOR) == HIGH); }

#pragma endregion

// =============================================================
#pragma region Homing
// =============================================================

void startHoming() {
	// If both home switches are already active, we are already at home — skip homing
	if (checkHomeSwitch1() && checkHomeSwitch2()) {
		motorsSetCurrentPosition(0);
		isHomingActive = false;
		Serial.println("Already homed — skipping homing sequence");
		if (isFirstBoot && digitalRead(SENSOR_PINS[CENTER_SENSOR_INDEX]) == LOW) {
			isFirstBoot = false;
			setError(ErrorCode::WAGON_AT_STARTUP, "Wagon Docked!");
			currentState = LoaderState::ERROR;
			return;
		}
		isFirstBoot  = false;
		isHomed      = true;
		currentState = LoaderState::IDLE;
		addStatusMessage("Homed");
		addStatusMessage("Waiting");
		return;
	}

	isHomingActive  = true;
	isHomed         = false;
	homingPhase     = HomingPhase::RETRACTING;
	homingM1Done   = checkHomeSwitch1();
	homingM2Done   = checkHomeSwitch2();
	homingStartTime = millis();
	armSyncTimer    = 0;
	currentState    = LoaderState::HOMING;
	stateTimer      = millis();

	if (homingM1Done) stepper->setCurrentPosition(0);
	else { stepper->setSpeedInHz((uint32_t)HOMING_SPEED); stepper->moveTo(-1000000L); }

	if (homingM2Done) stepper2->setCurrentPosition(0);
	else { stepper2->setSpeedInHz((uint32_t)HOMING_SPEED); stepper2->moveTo(-1000000L); }

	setMotorsEnabled(true);
	addStatusMessage("Homing...");
	Serial.println("Starting homing sequence...");
}

// Returns true when homing is complete
bool runHomingStateMachine() {
	switch (homingPhase) {

		case HomingPhase::RETRACTING:
			if (!homingM1Done && checkHomeSwitch1()) {
				stepper->forceStopAndNewPosition(0);
				stepper->setSpeedInHz((uint32_t)STEPPER_MAX_SPEED);
				homingM1Done = true;
				if (!homingM2Done && armSyncTimer == 0) armSyncTimer = millis(); // start timer; waiting on m2
				Serial.println("Motor 1 homed");
			}
			if (!homingM2Done && checkHomeSwitch2()) {
				stepper2->forceStopAndNewPosition(0);
				stepper2->setSpeedInHz((uint32_t)STEPPER_MAX_SPEED);
				homingM2Done = true;
				if (!homingM1Done && armSyncTimer == 0) armSyncTimer = millis(); // start timer; waiting on m1
				Serial.println("Motor 2 homed");
			}
			// Arm sync check: one arm homed but other hasn't followed within timeout
			if (armSyncTimer > 0 && !(homingM1Done && homingM2Done) &&
				(millis() - armSyncTimer > ARM_SYNC_TIMEOUT)) {
				if (!homingM1Done) { stepper->forceStopAndNewPosition(0);  stepper->setSpeedInHz((uint32_t)STEPPER_MAX_SPEED);  Serial.println("Motor 1 stuck"); }
				if (!homingM2Done) { stepper2->forceStopAndNewPosition(0); stepper2->setSpeedInHz((uint32_t)STEPPER_MAX_SPEED); Serial.println("Motor 2 stuck"); }
				setError(ErrorCode::ARM_SYNC, "Arm Stuck Home");
				homingPhase = HomingPhase::SETTLING;
				stepper->moveTo(HOMING_SETTLE_STEPS);
				stepper2->moveTo(HOMING_SETTLE_STEPS);
				armSyncTimer = 0;
				break;
			}
			// Check timeout
			if (millis() - homingStartTime >= HOMING_TIMEOUT) {
				if (!homingM1Done) { stepper->forceStopAndNewPosition(0);  stepper->setSpeedInHz((uint32_t)STEPPER_MAX_SPEED);  Serial.println("Motor 1 home timeout"); }
				if (!homingM2Done) { stepper2->forceStopAndNewPosition(0); stepper2->setSpeedInHz((uint32_t)STEPPER_MAX_SPEED); Serial.println("Motor 2 home timeout"); }
				setError(ErrorCode::HOMING_TIMEOUT, "Home Timeout");
				homingPhase = HomingPhase::SETTLING;
				stepper->moveTo(HOMING_SETTLE_STEPS);
				stepper2->moveTo(HOMING_SETTLE_STEPS);
				break;
			}
			if (homingM1Done && homingM2Done) {
				homingPhase  = HomingPhase::SETTLING;
				armSyncTimer = 0;
				stepper->moveTo(HOMING_SETTLE_STEPS);
				stepper2->moveTo(HOMING_SETTLE_STEPS);
			}
			break;

		case HomingPhase::SETTLING:
			if (motorsAtTarget()) {
				motorsSetCurrentPosition(0);
				homingPhase = HomingPhase::DONE;
			}
			break;

		case HomingPhase::DONE:
			isHomingActive = false;
			setMotorsEnabled(false);
			Serial.println("Homing complete!");
			if (currentError != ErrorCode::NONE) {
				// Error during homing (e.g. timeout) — motors parked, enter error state
				currentState = LoaderState::ERROR;
				addStatusMessage("Check System");
				return true;
			}
			if (isFirstBoot && digitalRead(SENSOR_PINS[CENTER_SENSOR_INDEX]) == LOW) {
				setError(ErrorCode::WAGON_AT_STARTUP, "Wagon Docked!");
				currentState = LoaderState::ERROR;
				return true;
			}
			isFirstBoot = false;
			isHomed = true;
			addStatusMessage("Homed");
			addStatusMessage("Waiting");
			return true;
	}
	return false;
}

#pragma endregion

// =============================================================
#pragma region Display
// =============================================================

void updateDisplay()
{
	if (millis() - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL) return;
	lastDisplayUpdate = millis();

	display.clearDisplay();
	display.setTextColor(SH110X_WHITE);
	display.setTextSize(1);

	// ── Error screen (overrides normal layout) ───────────────────────
	if (currentState == LoaderState::ERROR) {
		bool errBlink = (millis() % 500) < 250;
		if (errBlink) {
			display.fillRect(0, 0, 128, 64, SH110X_WHITE);
			display.setTextColor(SH110X_BLACK);
		}
		display.setTextSize(2);
		display.setCursor(34, 2);
		display.print(F("ERROR"));
		display.setTextSize(1);
		display.setCursor(0, 22);
		switch (currentError) {
			case ErrorCode::WAGON_AT_STARTUP:    display.print(F("Wagon at startup"));  break;
			case ErrorCode::STATE_TIMEOUT:       display.print(F("State timeout"));     break;
			case ErrorCode::HOMING_TIMEOUT:      display.print(F("Homing timeout"));    break;
			case ErrorCode::ARM_SYNC:            display.print(F("Arm out of sync"));   break;
			case ErrorCode::EXTEND_TIMEOUT:      display.print(F("Extend timeout"));    break;
			case ErrorCode::EXTEND_ARM_SYNC:     display.print(F("Arm sync extend"));   break;
			case ErrorCode::WAGON_NOT_DEPARTING: display.print(F("Wagon stuck"));       break;
			case ErrorCode::SENSOR_STUCK:        display.print(F("Sensor stuck"));      break;
			case ErrorCode::ESPNOW_TIMEOUT:      display.print(F("Comms lost"));        break;
			default:                              display.print(F("System error"));      break;
		}
		display.setCursor(0, 35);
		display.print(F("-> B: Clear + Re-home"));
		display.setCursor(0, 50);
		display.print(statusMessages[0]);
		display.setTextColor(SH110X_WHITE);
		display.display();
		return;
	}

	// ── Layout constants ─────────────────────────────────────────────
	//  y= 0.. 2  home stop-bar (8×3 T-bar above each shaft)
	//  y= 3..27  L-shaped arm shafts + feet
	//  y=28..35  wagon zone: can rect (28..30), body (28..34), wheels (33..35)
	//  y=36      single track rail
	//  y=43      status line 1 + direction/polarity mode (right-aligned)
	//  y=54      status line 2
	const int ARM_TOP_Y  = 3;   // shaft starts below the stop-bar
	const int ARM_BOT_Y  = 27;  // arm foot y when fully extended
	const int ARM_LX     = 35;  // left arm shaft x  (2 px wide: 35–36)
	const int ARM_RX     = 91;  // right arm shaft x (2 px wide: 91–92)
	const int ARM_FOOT_W = 15;  // horizontal foot length toward centre
	// Wagon shape dimensions — wagon undercarriage sits directly under the can
	const int WGN_CAN_W    = 16; // can body width (wagon is this wide)
	const int WGN_CAN_Y    = 28; // can body top
	const int WGN_CAN_H    = 6;  // can body height → bottom at y=33
	const int WGN_WHEEL_W  = 4;  // each wheel width
	const int WGN_WHEEL_H  = 3;  // wheel height → bottom at y=36 (on rail)
	const int RAIL_Y       = 36; // single track rail
	const int STAT1_Y    = 43;  // status line 1
	const int STAT2_Y    = 54;  // status line 2

	const int sensorCenters[5] = { 11, 32, 64, 96, 117 };
	// 6 gap zones: [0]=left of s0, [1]=s0–s1, [2]=s1–s2, [3]=s2–s3, [4]=s3–s4, [5]=right of s4
	const int gapCenters[6]    = { 0, 21, 48, 80, 106, 127 };

	// ── Top bar: state label + TX indicator ──────────────────────────
	display.setCursor(0, 1);
	switch (currentState) {
		case LoaderState::IDLE:        display.print(isHomed ? F("IDLE") : F("UHMD")); break;
		case LoaderState::IGNORING:    display.print(F("IGNR")); break;
		case LoaderState::APPROACHING: display.print(F("APPR")); break;
		case LoaderState::DOCKED:      display.print(F("DOCK")); break;
		case LoaderState::CORRECTING:  display.print(F("CORR")); break;
		case LoaderState::LOADING:     display.print(F("LOAD")); break;
		case LoaderState::DEPARTING:   display.print(F("DEPT")); break;
		case LoaderState::HOMING:      display.print(F("HOME")); break;
		case LoaderState::ERROR:       display.print(F("ERR!")); break;
	}
	bool isTransmitting = (millis() - lastCommandSent < 250);
	bool blinkOn        = (millis() % 250) < 125;
	if (espnowConnected) {
		if (isTransmitting && blinkOn) {
			display.fillRect(109, 0, 19, 9, SH110X_WHITE);
			display.setTextColor(SH110X_BLACK);
			display.setCursor(113, 1);
			display.print(F("TX"));
			display.setTextColor(SH110X_WHITE);
		} else {
			display.setCursor(113, 1);
			display.print(F("TX"));
		}
	} else {
		display.drawRect(109, 0, 19, 9, SH110X_WHITE);
	}

	// ── L-shaped loader arms ──────────────────────────────────────────
	long pos = constrain(stepper->getCurrentPosition(), 0, TOTAL_LOAD_STEPS);
	int  aY  = ARM_TOP_Y + (int)((long)pos * (ARM_BOT_Y - ARM_TOP_Y) / TOTAL_LOAD_STEPS);

	// Home stop-bar: 8×3 T-bar centred on each shaft.
	// Filled solid = home switch active (arm is home), outline only = not at home.
	bool sw1 = checkHomeSwitch1();
	bool sw2 = checkHomeSwitch2();
	if (sw1) display.fillRect(ARM_LX - 3, 0, 8, 3, SH110X_WHITE);
	else     display.drawRect(ARM_LX - 3, 0, 8, 3, SH110X_WHITE);
	if (sw2) display.fillRect(ARM_RX - 3, 0, 8, 3, SH110X_WHITE);
	else     display.drawRect(ARM_RX - 3, 0, 8, 3, SH110X_WHITE);

	// Left arm: 2-wide vertical shaft + 2-wide inward foot
	display.drawLine(ARM_LX,     ARM_TOP_Y, ARM_LX,     aY, SH110X_WHITE);
	display.drawLine(ARM_LX + 1, ARM_TOP_Y, ARM_LX + 1, aY, SH110X_WHITE);
	display.drawLine(ARM_LX,     aY,     ARM_LX + ARM_FOOT_W, aY,     SH110X_WHITE);
	display.drawLine(ARM_LX,     aY + 1, ARM_LX + ARM_FOOT_W, aY + 1, SH110X_WHITE);

	// Right arm: 2-wide vertical shaft + 2-wide inward foot
	display.drawLine(ARM_RX,     ARM_TOP_Y, ARM_RX,     aY, SH110X_WHITE);
	display.drawLine(ARM_RX + 1, ARM_TOP_Y, ARM_RX + 1, aY, SH110X_WHITE);
	display.drawLine(ARM_RX - ARM_FOOT_W, aY,     ARM_RX + 1, aY,     SH110X_WHITE);
	display.drawLine(ARM_RX - ARM_FOOT_W, aY + 1, ARM_RX + 1, aY + 1, SH110X_WHITE);

	// ── Wagon ─────────────────────────────────────────────────────────
	// Pixel X from vehiclePosition or estimatedGapIndex
	int wagonX = -1;
	if (vehiclePosition >= 1.0f && vehiclePosition <= 5.0f) {
		wagonX = sensorCenters[0] + (int)((vehiclePosition - 1.0f) / 4.0f * (sensorCenters[4] - sensorCenters[0]));
	} else if (estimatedGapIndex >= 0 && estimatedGapIndex < 6) {
		wagonX = gapCenters[estimatedGapIndex];
	}

	if (wagonX >= 0) {
		bool can = isCanLoaded();
		int wx   = constrain(wagonX - WGN_CAN_W / 2, 0, 128 - WGN_CAN_W);

		// Can body: filled solid when can present, outlined when bay is empty
		if (can) display.fillRect(wx, WGN_CAN_Y, WGN_CAN_W, WGN_CAN_H, SH110X_WHITE);
		else     display.drawRect(wx, WGN_CAN_Y, WGN_CAN_W, WGN_CAN_H, SH110X_WHITE);

		// Wagon undercarriage: narrow platform below the can, same width
		int uy = WGN_CAN_Y + WGN_CAN_H; // top of undercarriage (y=34)
		display.drawLine(wx, uy, wx + WGN_CAN_W - 1, uy, SH110X_WHITE); // top edge
		display.drawLine(wx, uy + 1, wx + WGN_CAN_W - 1, uy + 1, SH110X_WHITE); // bottom edge

		// Wheels: two filled rectangles below the undercarriage, sitting on the rail
		display.fillRect(wx + 1,                      uy + 2, WGN_WHEEL_W, WGN_WHEEL_H, SH110X_WHITE); // left wheel
		display.fillRect(wx + WGN_CAN_W - 1 - WGN_WHEEL_W, uy + 2, WGN_WHEEL_W, WGN_WHEEL_H, SH110X_WHITE); // right wheel
	}

	// ── Single track rail ─────────────────────────────────────────────
	display.drawLine(0, RAIL_Y, 127, RAIL_Y, SH110X_WHITE);

	// ── Status line 1: message + direction/polarity mode (right-aligned) ──
	String dirBase;
	if      (currentDirectionMode == DirectionMode::BOTH)   dirBase = "<->";
	else if (currentDirectionMode == DirectionMode::L_TO_R) dirBase = " ->";
	else                                                      dirBase = "<- ";
	String modeStr = (commandPolarity == 1) ? dirBase + "+" : "+" + dirBase;
	int modeX = 128 - (int)modeStr.length() * 6; // 6 px per char at textSize=1

	display.setCursor(0, STAT1_Y);
	display.print(rotatingSymbols[symbolIndex]);
	symbolIndex = (symbolIndex + 1) % 4;
	display.setCursor(8, STAT1_Y);
	display.print(statusMessages[0]);
	display.setCursor(modeX, STAT1_Y);
	display.print(modeStr);

	// ── Status line 2 ─────────────────────────────────────────────────
	display.setCursor(0, STAT2_Y);
	display.print(statusMessages[1]);

	display.display();
}

#pragma endregion

// =============================================================
#pragma region Diagnostics
// =============================================================

void serialLog(const char* msg) {
	Serial.print("["); Serial.print(millis()); Serial.print("] "); Serial.println(msg);
}

int countActiveSensors(bool sensors[]) {
	int count = 0;
	for (int i = 0; i < 5; i++) { if (sensors[i]) count++; }
	return count;
}

void printSensorDiagnostics(bool sensors[]) {
	Serial.print("["); Serial.print(millis()); Serial.print("] Sensors: [");
	for (int i = 0; i < 5; i++) {
		Serial.print(sensors[i] ? "1" : "0");
		if (i < 4) Serial.print(",");
	}
	Serial.print("] Center: ");
	Serial.print(sensors[CENTER_SENSOR_INDEX] ? "YES" : "NO");
	Serial.print(" Count: ");
	Serial.println(countActiveSensors(sensors));
}

#pragma endregion