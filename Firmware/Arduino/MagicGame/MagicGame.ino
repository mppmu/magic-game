// File: MagicGame.ino
// Auth: M. Fras, Electronics Division, MPI for Physics, Munich
// Mod.: M. Fras, Electronics Division, MPI for Physics, Munich
// Date: 25 Nov 2022
// Rev.: 26 Apr 2023
//
// Firmware for the Arduino Mega 2560 Rev 3 to control the telescope model of
// the MAGIC Game via the MAGIC Game board.
//
// Stepper motors:
// - Azimuth: 28BYJ-48 with 32 steps and 1/64 gearing
// - Elevation: Adafruit 858 with 32 steps and 1/16 gearing
// - Directions:
//   - Azimuth left:    -1 step
//   - Azimuth right:   +1 step
//   - Elevation down:  -1 step
//   - Elevation up:    +1 step
//



#include <Stepper.h>
#include <LiquidCrystal.h>



#define FW_NAME         "MagicGame"
#define FW_VERSION      "0.0.22"
#define FW_RELEASEDATE  "26 Apr 2023"



// Pre-defined configurations.
// CAUTION: ONLY ONE CONFIGURATION MUST BE SELECTED AT A TIME!
// 1. Simulation with SimulIDE.
// 2. Bench top operation using the analog Joy-IT KY-023 joystick module.
// 3. Exhibition booth operation using the Sanwa JLF-TP-8YT-K digital joystick
//    in connection with the Joystick Adapter board V1.0 or V1.1.
//#define CONFIGURATION_SIMULATION
//#define CONFIGURATION_BENCH_TOP_OPERATION
#define CONFIGURATION_EXHIBITION_BOOTH_OPERATION

// For simulation with SimulIDE.
//#define SIMULATION_MODE
#ifdef CONFIGURATION_SIMULATION
#define SIMULATION_MODE
#endif

// For debugging.
//#define DEBUG_MODE_SHOW_STEPS
//#define DEBUG_MODE_SHOW_POSITIONS

// Debugging over the serial interface.
#define DEBUG_SERIAL

// Define serial interfaces.
#define SERIAL_CONSOLE                      Serial
#define SERIAL_CONTROL                      Serial1

// Define the line ending string for console interface.
#define CONSOLE_MSG_EOL                     "\r\n"

// Define control messages for communication with the exhibition booth.
#define ENABLE_CONTROL_MSG
#define ENABLE_CONTROL_MSG_PROGRESS
#define ENABLE_CONTROL_MSG_COUNTDOWN
#define ENABLE_CONTROL_MSG_IN_GAME_EVAL
#define ENABLE_CONTROL_MSG_IN_GAME_EVAL_DIFF_ONLY
#define CONTROL_MSG_EOL                     "\r\n"
#define CONTROL_MSG_ERROR                   "ERROR"
#define CONTROL_MSG_BOOT                    "BOOT"
#define CONTROL_MSG_INIT                    "INIT"
#define CONTROL_MSG_IDLE                    "IDLE"
#define CONTROL_MSG_START                   "START"
#define CONTROL_MSG_TARGET                  "TARGET"
#define CONTROL_MSG_TARGET_MATCH            "O"
#define CONTROL_MSG_TARGET_MISS             "X"
#define CONTROL_MSG_PROGRESS                "PROGRESS"
#define CONTROL_MSG_COUNTDOWN               "COUNTDOWN"
#define CONTROL_MSG_SUCCESS                 "SUCCESS"
#define CONTROL_MSG_FAILURE                 "FAILURE"
#define CONTROL_CMD_RESET                   "RESET"

// Define number of decimals in serial messages.
#define SERIAL_MSG_DECIMALS_AZIMUTH         1
#define SERIAL_MSG_DECIMALS_ELEVATION       1

// Define degree symbol.
// Note: The degree symbol ° is often not displayed correctly in terminal
//       programs. So it may be better to omit it.
//const String stringDegree = "°";
const String stringDegree = "";

// Custom type for telescope coordinates.
typedef struct {
  float azimuth;
  float elevation;
} coordinate_t;

// Find the zero positions for azimuth and elevation using the limit switches.
// This is required to calibrate the absolute position of the telescope.
// THIS MUST BE ENABLED FOR NORMAL USE!
#define FIND_ZERO_POSITIONS

// Check if a wrong limit switch gets activated while finding the zero positions.
// This should be activated for normal use.
#define FIND_ZERO_POS_CHECK_WRONG_LIMIT_SW

// Initialize the hardware before every game. This includes finding the zero positions for azimuth and elevation using the limit switches.
// This should be activated for normal use to avoid a potential accumulation of lost steps.
#define INIT_HARDWARE_BEFORE_EVERY_GAME

// Move the telescope to its parking position before starting the game.
#define MOVE_TELESCOPE_TO_PARKING_POSITION

// Show the stepper motor steps when moving the telescope to a given position.
#define MOVE_TELESCOPE_SHOW_MOTOR_STEPS

// Use random seed from an analogue input to generate fully random numbers for
// the targets.
#define USE_RANDOM_SEED_FROM_ANALOG_INPUT

// Use fixed, predefined targets only. Otherwise use random targets.
// Note: For the operation in the exhibition booth, 3 fixed targets are required.
#define USE_FIXED_TARGETS

// Define phyiscal limits for telescope movement.
#define PHYSICAL_LIMIT_AZIMUTH_LEFT         0
//#define PHYSICAL_LIMIT_AZIMUTH_RIGHT        285     // Maximum range between the limit switches.
#define PHYSICAL_LIMIT_AZIMUTH_RIGHT        220     // Maximum physical range supported by the telescope.
#define PHYSICAL_LIMIT_ELEVATION_BOTTOM     35
#define PHYSICAL_LIMIT_ELEVATION_TOP        145

// Define timeouts (in ms) for automatically proceeding.
#define TIMEOUT_BOOT_FIND_ZERO_POSITIONS    5000    // Timeout between power up and finding the zero positions.
#define TIMEOUT_PREPARE_NEW_GAME            10000   // Timeout to prepare a new game at the end of the finished game.

// Swap azimuth and elevation axis of the joystick.
// Note: This is required for the Sanwa JLF-TP-8YT-K joystick connected via the
//       Joystick Adapter board V1.0 or V1.1, if the Sanwa JLF-TP-8YT-K
//       joystick is mounted with its connector on the right side like in the
//       exhibition booth (top view):
//
//                EL UP
//               +-------+
//               |       |
//               |       |
//       AZ L <- |   o   | -> AZ R
//               |       ==== connector with cable
//               |       |
//               +-------+
//               EL DOWN
//
//       No change is needed if the Sanwa JLF-TP-8YT-K joystick is mounted with
//       its connector on the top side (top view):
//
//                    connector with cable
//                       ||
//                 EL UP ||
//               +-------||--+
//               |           |
//       AZ L <- |     o     | -> AZ R
//               |           |
//               +-----------+
//                  EL DOWN
//
//#define JOYSTICK_SWAP_AZIMUTH_ELEVATION

// Invert the azimuth axis of the joystick.
//#define JOYSTICK_INVERT_AZIMUTH

// Invert the elevation axis of the joystick.
// Note: This is also required for the Sanwa JLF-TP-8YT-K joystick connected
//       via the Joystick Adapter board V1.0, if the Sanwa JLF-TP-8YT-K
//       joystick is mounted with its connector on the right side like in the
//       exhibition booth (top view). For details see the explanation above.
//#define JOYSTICK_INVERT_ELEVATION

// Do not end the game, but stay in an infinite loop.
// FOR TESTING ONLY!
//#define INFINITE_GAME_LOOP

// Ignore soft limits when moving the telescope.
// FOR TESTING ONLY!
//#define IGNORE_SOFT_LIMITS_AZIMUTH
//#define IGNORE_SOFT_LIMITS_ELEVATION



// Define pins.
#ifdef JOYSTICK_SWAP_AZIMUTH_ELEVATION
#define PIN_JOYSTICK_AZ                     A1
#define PIN_JOYSTICK_EL                     A0
#else
#define PIN_JOYSTICK_AZ                     A0
#define PIN_JOYSTICK_EL                     A1
#endif
#define PIN_JOYSTICK_BUTTON                 A2
#define PIN_MOTOR_AZIMUTH_A_P               30
#define PIN_MOTOR_AZIMUTH_A_N               32
#define PIN_MOTOR_AZIMUTH_B_P               31
#define PIN_MOTOR_AZIMUTH_B_N               33
#define PIN_MOTOR_ELEVATION_A_P             34
#define PIN_MOTOR_ELEVATION_A_N             36
#define PIN_MOTOR_ELEVATION_B_P             35
#define PIN_MOTOR_ELEVATION_B_N             37
#define PIN_LED_PROGRESS_1                  2
#define PIN_LED_PROGRESS_2                  3
#define PIN_LED_PROGRESS_3                  4
#define PIN_LED_PROGRESS_4                  5
#define PIN_LED_PROGRESS_5                  6
#define PIN_SW_LIMIT_AZIMUTH_LEFT           22
#define PIN_SW_LIMIT_AZIMUTH_RIGHT          23
#define PIN_SW_LIMIT_ELEVATION_BOTTOM       24
#define PIN_SW_LIMIT_ELEVATION_TOP          25
#define PIN_BUTTON_START                    26
#define PIN_LED_AZIMUTH_LEFT                38
#define PIN_LED_AZIMUTH_CENTER              39
#define PIN_LED_AZIMUTH_RIGHT               40
#define PIN_LED_ELEVATION_BOTTOM            42
#define PIN_LED_ELEVATION_CENTER            43
#define PIN_LED_ELEVATION_TOP               44
#define PIN_LED_OK                          46
#define PIN_LED_ERROR                       47
#define PIN_LCD_RS                          48
#define PIN_LCD_EN                          49
#define PIN_LCD_D4                          50
#define PIN_LCD_D5                          51
#define PIN_LCD_D6                          52
#define PIN_LCD_D7                          53
#define PIN_LCD_CONTRAST                    8
#define PIN_LCD_BACKLIGHT                   9



// Stepper motors: Define steps per revolution and speed in RPM.
#ifdef SIMULATION_MODE
#define STEPS_AZIMUTH                       64      // Steps per revolution.
#define SPEED_AZIMUTH                       10      // RPM.
#define AZIMUTH_DEGREES_PER_REVOLUTION      360     // Degrees of azimuth per stepper motor revolution.
#define AZIMUTH_REVERSE_DIRECTION
#define STEPS_ELEVATION                     64      // Steps per revolution.
#define SPEED_ELEVATION                     10      // RPM.
#define ELEVATION_DEGREES_PER_REVOLUTION    90      // Degrees of elevation per stepper motor revolution.
#define ELEVATION_REVERSE_DIRECTION
#else
// Stepper motor for azimuth:
// - Stride angle: 11.25°/64
// - Steps per revolution: 2048 (empirically tested)
#define STEPS_AZIMUTH                       2048    // Steps per revolution.
#define SPEED_AZIMUTH                       5       // RPM.
#define AZIMUTH_DEGREES_PER_REVOLUTION      360     // Degrees of azimuth per stepper motor revolution.
#define AZIMUTH_REVERSE_DIRECTION
// Stepper motor for elevation:
// - Stride angle: 11.25°/16.128
// - Steps per revolution: 516
#define STEPS_ELEVATION                     516     // Steps per revolution.
#define SPEED_ELEVATION                     20      // RPM.
#define ELEVATION_DEGREES_PER_REVOLUTION    29.0    // Degrees of elevation per stepper motor revolution.
//#define ELEVATION_REVERSE_DIRECTION
#endif // SIMULATION_MODE

// Create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to.
#ifdef AZIMUTH_REVERSE_DIRECTION
Stepper stepperAzimuth(STEPS_AZIMUTH, PIN_MOTOR_AZIMUTH_B_N, PIN_MOTOR_AZIMUTH_B_P, PIN_MOTOR_AZIMUTH_A_N, PIN_MOTOR_AZIMUTH_A_P);
#else
Stepper stepperAzimuth(STEPS_AZIMUTH, PIN_MOTOR_AZIMUTH_A_P, PIN_MOTOR_AZIMUTH_A_N, PIN_MOTOR_AZIMUTH_B_P, PIN_MOTOR_AZIMUTH_B_N);
#endif
#ifdef ELEVATION_REVERSE_DIRECTION
Stepper stepperElevation(STEPS_ELEVATION, PIN_MOTOR_ELEVATION_B_N, PIN_MOTOR_ELEVATION_B_P, PIN_MOTOR_ELEVATION_A_N, PIN_MOTOR_ELEVATION_A_P);
#else
Stepper stepperElevation(STEPS_ELEVATION, PIN_MOTOR_ELEVATION_A_P, PIN_MOTOR_ELEVATION_A_N, PIN_MOTOR_ELEVATION_B_P, PIN_MOTOR_ELEVATION_B_N);
#endif
const int millisPerStepAzimuth = (float) 60 / (STEPS_AZIMUTH * SPEED_AZIMUTH) * 1000;
const int millisPerStepElevation = (float) 60 / (STEPS_ELEVATION * SPEED_ELEVATION) * 1000;
long positionStepsAzimuth = 0;
long positionStepsElevation = 0;

// Azimuth and elevation software limits.
#ifdef SIMULATION_MODE
const float azimuthLimitMin     = 10.0;
const float azimuthLimitMax     = 350.0;
const float elevationLimitMin   = 10.0;
const float elevationLimitMax   = 80.0;
#else
const float azimuthLimitMin     = PHYSICAL_LIMIT_AZIMUTH_LEFT + 10.0;
const float azimuthLimitMax     = PHYSICAL_LIMIT_AZIMUTH_RIGHT - 10.0;
const float elevationLimitMin   = PHYSICAL_LIMIT_ELEVATION_BOTTOM + 5.0;
const float elevationLimitMax   = 90;
#endif

// Azimuth and elevation positions.
#ifdef SIMULATION_MODE
const float azimuthPosParking   = 20.0;
const float elevationPosParking = 20.0;
#else
const float azimuthPosParking   = azimuthLimitMin;
const float elevationPosParking = elevationLimitMin;
#endif

// Azimuth and elevation actual value, target value and tolerance.
#ifdef SIMULATION_MODE
float azimuthActual             = 0.0;
float azimuthTarget             = 0.0;
const float azimuthTolerance    = 10.0;
float elevationActual           = 0.0;
float elevationTarget           = 0.0;
const float elevationTolerance  = 10.0;
#else
float azimuthActual             = 0.0;
float azimuthTarget             = 0.0;
const float azimuthTolerance    = 5.0;
float elevationActual           = 0.0;
float elevationTarget           = 0.0;
const float elevationTolerance  = 5.0;
#endif
// Define 3 fixed targets for operation in the exhibition booth.
#ifdef USE_FIXED_TARGETS
int fixedTargetSel;
const coordinate_t fixedTargets[] = {
//  {160.0, 70.0},
//  {180.0, 85.0},
//  {200.0, 80.0}
  // Mean value of the positions empirically determined by five persons.
  {107.9, 52.0},
  {149.2, 58.9},
  {201.5, 59.9}
};
#endif

// End the game as soon as the target position is reached.
//#define END_GAME_AT_TARGET_POSITION

// Automatically move the telescope between given coordinates for testing.
// FOR TESTING ONLY!
//#define AUTO_MOVE_LOOP
#ifdef AUTO_MOVE_LOOP
const coordinate_t autoMoveCoordinates[] = {
  {azimuthLimitMin, elevationLimitMin},         // Lower left position.
  {azimuthLimitMax, elevationLimitMax}          // Upper right position.
};
#endif



// Create an instance of the LCD.
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);



// Settings based on pre-defined configurations.
#ifdef CONFIGURATION_SIMULATION
#define SIMULATION_MODE
#undef DEBUG_MODE_SHOW_STEPS
#undef DEBUG_MODE_SHOW_POSITIONS
#undef DEBUG_SERIAL
#define ENABLE_CONTROL_MSG
#define ENABLE_CONTROL_MSG_PROGRESS
#define ENABLE_CONTROL_MSG_COUNTDOWN
#define ENABLE_CONTROL_MSG_IN_GAME_EVAL
#define ENABLE_CONTROL_MSG_IN_GAME_EVAL_DIFF_ONLY
#undef FIND_ZERO_POSITIONS
#undef FIND_ZERO_POS_CHECK_WRONG_LIMIT_SW
#undef INIT_HARDWARE_BEFORE_EVERY_GAME
#define MOVE_TELESCOPE_TO_PARKING_POSITION
#define USE_RANDOM_SEED_FROM_ANALOG_INPUT
#undef USE_FIXED_TARGETS
#undef END_GAME_AT_TARGET_POSITION
#undef JOYSTICK_SWAP_AZIMUTH_ELEVATION
#undef PIN_JOYSTICK_AZ
#undef PIN_JOYSTICK_EL
#define PIN_JOYSTICK_AZ                     A0
#define PIN_JOYSTICK_EL                     A1
#undef JOYSTICK_INVERT_AZIMUTH
#undef JOYSTICK_INVERT_ELEVATION
#undef INFINITE_GAME_LOOP
#undef IGNORE_SOFT_LIMITS_AZIMUTH
#undef IGNORE_SOFT_LIMITS_ELEVATION
#endif  // CONFIGURATION_SIMULATION

#ifdef CONFIGURATION_BENCH_TOP_OPERATION
#undef SIMULATION_MODE
#undef DEBUG_MODE_SHOW_STEPS
#undef DEBUG_MODE_SHOW_POSITIONS
#undef DEBUG_SERIAL
#define ENABLE_CONTROL_MSG
#define ENABLE_CONTROL_MSG_PROGRESS
#define ENABLE_CONTROL_MSG_COUNTDOWN
#define ENABLE_CONTROL_MSG_IN_GAME_EVAL
#define ENABLE_CONTROL_MSG_IN_GAME_EVAL_DIFF_ONLY
#define FIND_ZERO_POSITIONS
#define FIND_ZERO_POS_CHECK_WRONG_LIMIT_SW
#define INIT_HARDWARE_BEFORE_EVERY_GAME
#define MOVE_TELESCOPE_TO_PARKING_POSITION
#define USE_RANDOM_SEED_FROM_ANALOG_INPUT
#undef USE_FIXED_TARGETS
#undef END_GAME_AT_TARGET_POSITION
#undef JOYSTICK_SWAP_AZIMUTH_ELEVATION
#undef PIN_JOYSTICK_AZ
#undef PIN_JOYSTICK_EL
#define PIN_JOYSTICK_AZ                     A0
#define PIN_JOYSTICK_EL                     A1
#define JOYSTICK_INVERT_AZIMUTH
#undef JOYSTICK_INVERT_ELEVATION
#undef INFINITE_GAME_LOOP
#undef IGNORE_SOFT_LIMITS_AZIMUTH
#undef IGNORE_SOFT_LIMITS_ELEVATION
#endif  // CONFIGURATION_BENCH_TOP_OPERATION

#ifdef CONFIGURATION_EXHIBITION_BOOTH_OPERATION
#undef SIMULATION_MODE
#undef DEBUG_MODE_SHOW_STEPS
#undef DEBUG_MODE_SHOW_POSITIONS
#undef DEBUG_SERIAL
#define ENABLE_CONTROL_MSG
#undef ENABLE_CONTROL_MSG_PROGRESS
#define ENABLE_CONTROL_MSG_COUNTDOWN
#define ENABLE_CONTROL_MSG_IN_GAME_EVAL
#define ENABLE_CONTROL_MSG_IN_GAME_EVAL_DIFF_ONLY
#define FIND_ZERO_POSITIONS
#define FIND_ZERO_POS_CHECK_WRONG_LIMIT_SW
#define INIT_HARDWARE_BEFORE_EVERY_GAME
#define MOVE_TELESCOPE_TO_PARKING_POSITION
#define USE_RANDOM_SEED_FROM_ANALOG_INPUT
#define USE_FIXED_TARGETS
#define END_GAME_AT_TARGET_POSITION
#define JOYSTICK_SWAP_AZIMUTH_ELEVATION
#undef PIN_JOYSTICK_AZ
#undef PIN_JOYSTICK_EL
#define PIN_JOYSTICK_AZ                     A1
#define PIN_JOYSTICK_EL                     A0
#define JOYSTICK_INVERT_AZIMUTH
#define JOYSTICK_INVERT_ELEVATION
#undef INFINITE_GAME_LOOP
#undef IGNORE_SOFT_LIMITS_AZIMUTH
#undef IGNORE_SOFT_LIMITS_ELEVATION
#endif  // CONFIGURATION_EXHIBITION_BOOTH_OPERATION



void setup() {
  // Initialize the serial interfaces.
  SERIAL_CONSOLE.begin(9600);       // Console interface for debugging.
  SERIAL_CONTROL.begin(9600);       // Control interface for communication with exhibition booth.
  // Send control message.
  #ifdef ENABLE_CONTROL_MSG
  SERIAL_CONTROL.print(String(CONTROL_MSG_EOL));
  SERIAL_CONTROL.print(String(CONTROL_MSG_BOOT) + String(CONTROL_MSG_EOL));
  #endif
  // Print a message on the serial console.
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "*** Welcome to the MAGIC Game: Catch the Gamma! ***");
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Firmware info:");
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "- Name: ");
  SERIAL_CONSOLE.print(FW_NAME);
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "- Ver.: ");
  SERIAL_CONSOLE.print(FW_VERSION);
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "- Date: ");
  SERIAL_CONSOLE.print(FW_RELEASEDATE);
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));

  // Set the speed of the stepper motors.
  stepperAzimuth.setSpeed(SPEED_AZIMUTH);
  stepperElevation.setSpeed(SPEED_ELEVATION);

  // Set the LCD contrast.
  pinMode(PIN_LCD_CONTRAST, OUTPUT);
  analogWrite(PIN_LCD_CONTRAST, 0x00);
  // Set the LCD backlight brightness.
  pinMode(PIN_LCD_BACKLIGHT, OUTPUT);
  analogWrite(PIN_LCD_BACKLIGHT, 0xff);
  // Set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message on the LCD.
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MAGIC Game:");
  lcd.setCursor(0, 1);
  lcd.print("Catch the Gamma!");
  delay(1000);

  // Set up limit switches.
  pinMode(PIN_SW_LIMIT_AZIMUTH_LEFT, INPUT_PULLUP);
  pinMode(PIN_SW_LIMIT_AZIMUTH_RIGHT, INPUT_PULLUP);
  pinMode(PIN_SW_LIMIT_ELEVATION_BOTTOM, INPUT_PULLUP);
  pinMode(PIN_SW_LIMIT_ELEVATION_TOP, INPUT_PULLUP);

  // Set up start buttons.
  pinMode(PIN_BUTTON_START, INPUT_PULLUP);
  pinMode(PIN_JOYSTICK_BUTTON, INPUT_PULLUP); // Alternative start button on joystick.

  // Set up the LEDs.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LED_PROGRESS_1, OUTPUT);
  pinMode(PIN_LED_PROGRESS_2, OUTPUT);
  pinMode(PIN_LED_PROGRESS_3, OUTPUT);
  pinMode(PIN_LED_PROGRESS_4, OUTPUT);
  pinMode(PIN_LED_PROGRESS_5, OUTPUT);
  pinMode(PIN_LED_AZIMUTH_LEFT, OUTPUT);
  pinMode(PIN_LED_AZIMUTH_CENTER, OUTPUT);
  pinMode(PIN_LED_AZIMUTH_RIGHT, OUTPUT);
  pinMode(PIN_LED_ELEVATION_BOTTOM, OUTPUT);
  pinMode(PIN_LED_ELEVATION_CENTER, OUTPUT);
  pinMode(PIN_LED_ELEVATION_TOP, OUTPUT);
  pinMode(PIN_LED_OK, OUTPUT);
  pinMode(PIN_LED_ERROR, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);    // Show power on status.
  digitalWrite(PIN_LED_PROGRESS_1, LOW);
  digitalWrite(PIN_LED_PROGRESS_2, LOW);
  digitalWrite(PIN_LED_PROGRESS_3, LOW);
  digitalWrite(PIN_LED_PROGRESS_4, LOW);
  digitalWrite(PIN_LED_PROGRESS_5, LOW);
  digitalWrite(PIN_LED_AZIMUTH_LEFT, LOW);
  digitalWrite(PIN_LED_AZIMUTH_CENTER, LOW);
  digitalWrite(PIN_LED_AZIMUTH_RIGHT, LOW);
  digitalWrite(PIN_LED_ELEVATION_BOTTOM, LOW);
  digitalWrite(PIN_LED_ELEVATION_CENTER, LOW);
  digitalWrite(PIN_LED_ELEVATION_TOP, LOW);
  digitalWrite(PIN_LED_ELEVATION_TOP, LOW);
  digitalWrite(PIN_LED_OK, LOW);
  digitalWrite(PIN_LED_ERROR, LOW);

  // Set random seed.
  #ifdef USE_RANDOM_SEED_FROM_ANALOG_INPUT
  randomSeed(analogRead(PIN_JOYSTICK_AZ));  // Fully random numbers based on analog input.
  #else
  randomSeed(0);
  #endif

  // Wait until the start button is pushed. Alternatively use a timeout.
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Please press the start button to continue (timeout in " + String(TIMEOUT_BOOT_FIND_ZERO_POSITIONS / 1000) + " seconds).");
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));
  waitStart(TIMEOUT_BOOT_FIND_ZERO_POSITIONS);
  delay(1000);
}



void loop() {

  // Initialize the hardware.
  initHardware();

  // Automatically move the telescope between given coordinates for testing.
  #ifdef AUTO_MOVE_LOOP
  autoMoveLoop();
  #endif

  while (true) {
    // Initialize the game.
    initGame();
    // Play the game.
    playGame();
  }
}



// Error handling.
// => Halt the execution and display an error message.
int errorHandler(String errorMsgSerial, String errorMsgLcd) {
  String stringSerial;
  // Switch off OK LED and turn on error LED.
  digitalWrite(PIN_LED_OK, LOW);
  digitalWrite(PIN_LED_ERROR, HIGH);
  // Power down the stepper motors to save power and keep them cool.
  stepperPowerDownAzimuth();
  stepperPowerDownElevation();
  // Send error message to serial interface and LCD.
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));
  SERIAL_CONSOLE.print(errorMsgSerial);
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Press the reset button to reboot.");
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));
  lcd.clear();
  while (true) {
    // Send error message to serial control interface.
    #ifdef ENABLE_CONTROL_MSG
    SERIAL_CONTROL.print(String(CONTROL_MSG_ERROR) + String(CONTROL_MSG_EOL));
    #endif
    // Display error message on LCD.
    lcd.setCursor(0, 0);
    lcd.print(errorMsgLcd);
    lcd.setCursor(0, 1);
    lcd.print("PROGRAM STOPPED!");
    delay(1000);
    lcd.setCursor(0, 1);
    lcd.print("PRESS RESET!    ");
    delay(1000);
    // Read from serial console and check if a program reset was requested.
    stringSerial = SERIAL_CONSOLE.readStringUntil('\n');
    stringSerial.replace("\r", "");
    if (stringSerial.equalsIgnoreCase(CONTROL_CMD_RESET)) {
      reset();
    }
    // Read from serial control interface and check if a program reset was requested.
    #ifdef ENABLE_CONTROL_MSG
    stringSerial = SERIAL_CONTROL.readStringUntil('\n');
    stringSerial.replace("\r", "");
    if (stringSerial.equalsIgnoreCase(CONTROL_CMD_RESET)) {
      reset();
    }
    #endif
  }
  return 0;
}



// Soft reset function.
void reset(void) {
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Resetting the Arduino via software.");
  delay(500);
  asm volatile ("jmp 0");
}



// Wait until the start button is pushed or a timeout occurs.
// Note: A timeout value of 0 ms or below means no timeout.
int waitStart(long timeout_ms) {
  long timeStart;
  long timeElapsed;

  timeStart = millis();

  while (digitalRead(PIN_BUTTON_START) && digitalRead(PIN_JOYSTICK_BUTTON)) {
    if (timeout_ms > 0) {
      timeElapsed = millis() - timeStart;
      if (timeElapsed > timeout_ms) return 0;
    }
  };
  return 0;
}



// Initialize the hardware.
int initHardware() {
  int ret = 0;

  #ifdef ENABLE_CONTROL_MSG
  SERIAL_CONTROL.print(String(CONTROL_MSG_INIT) + String(CONTROL_MSG_EOL));
  #endif

  // Find the zero positions of the stepper motors.
  #ifdef FIND_ZERO_POSITIONS
  // Do not try to find the zero positions of the stepper motors in simulation mode.
  #ifndef SIMULATION_MODE
  ret = stepperFindZeroPosition();
  // Error while finding the zero positions.
  if (ret) {
    errorHandler(String(CONSOLE_MSG_EOL) + "ERROR: Zero position of stepper motors not found! Program stopped!", "ERROR: Zero pos.");
  }
  #endif
  #endif

  // Power down the stepper motors to save power and keep them cool.
  stepperPowerDownAzimuth();
  stepperPowerDownElevation();

  digitalWrite(PIN_LED_OK, HIGH);

  return 0;
}



// Drive the stepper motors to their limit switch in order to get the zero positions.
int stepperFindZeroPosition() {
  long steps;
  // Display message.
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Finding the zero positions of the stepper motors.");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Find zero pos.: ");
  // Azimuth: Move to left until limit switch gets activated.
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "- Azimuth... ");
  lcd.setCursor(0, 1);
  lcd.print("Azimuth...      ");
  steps = 0;
  #ifdef FIND_ZERO_POS_CHECK_WRONG_LIMIT_SW
  // Move 5 degrees left to avoid a false error if the telescope is at the azimuth right limit position.
  stepperAzimuth.step((-5 / AZIMUTH_DEGREES_PER_REVOLUTION) * STEPS_AZIMUTH);
  // Move 5 degrees down to avoid a false error if the telescope is at the elevation to limit position.
  stepperElevation.step((-5 / ELEVATION_DEGREES_PER_REVOLUTION) * STEPS_ELEVATION);
  #endif
  while (digitalRead(PIN_SW_LIMIT_AZIMUTH_LEFT)) {
    stepperAzimuth.step(-1);    // Move one step left.
    steps++;
    // Error: One complete rotation without limit switch getting activated!
    if (steps > (360 / AZIMUTH_DEGREES_PER_REVOLUTION) * STEPS_AZIMUTH) {
      SERIAL_CONSOLE.print("FAILED!");
      lcd.setCursor(0, 1);
      lcd.print("Azimuth FAILED! ");
      return 1;
    }
    // Error: The wrong limit switch got activated.
    #ifdef FIND_ZERO_POS_CHECK_WRONG_LIMIT_SW
    //if (!digitalRead(PIN_SW_LIMIT_AZIMUTH_RIGHT) || !digitalRead(PIN_SW_LIMIT_ELEVATION_BOTTOM) || !digitalRead(PIN_SW_LIMIT_ELEVATION_TOP)) {
    // Don't check the limit switch for the elevation bottom to avoid a
    // potential false error when the telescope is in the elevation zero
    // position.
    if (!digitalRead(PIN_SW_LIMIT_AZIMUTH_RIGHT) || !digitalRead(PIN_SW_LIMIT_ELEVATION_TOP)) {
      SERIAL_CONSOLE.print("FAILED!");
      lcd.setCursor(0, 1);
      lcd.print("Azimuth FAILED! ");
      return 2;
    }
    #endif
  }
  SERIAL_CONSOLE.print("OK.");
  // Elevation: Move down until limit switch gets activated.
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "- Elevation... ");
  lcd.setCursor(0, 1);
  lcd.print("Elevation...    ");
  steps = 0;
  while (digitalRead(PIN_SW_LIMIT_ELEVATION_BOTTOM)) {
    stepperElevation.step(-1);  // Move one step down.
    steps++;
    // Error: Half a rotation without limit switch getting activated!
    if (steps > (180 / ELEVATION_DEGREES_PER_REVOLUTION) * STEPS_ELEVATION) {
      SERIAL_CONSOLE.print("FAILED!");
      lcd.setCursor(0, 1);
      lcd.print("Elevation FAILED");
      return 3;
    }
    // Error: The wrong limit switch got activated.
    #ifdef FIND_ZERO_POS_CHECK_WRONG_LIMIT_SW
    // Caution! Then limit switch for azimuth left position stays activated! So we cannot check for it here.
    if (!digitalRead(PIN_SW_LIMIT_AZIMUTH_RIGHT) || !digitalRead(PIN_SW_LIMIT_ELEVATION_TOP)) {
      SERIAL_CONSOLE.print("FAILED!");
      lcd.setCursor(0, 1);
      lcd.print("Elevation FAILED");
      return 4;
    }
    #endif
  }
  SERIAL_CONSOLE.print("OK.");
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));

  lcd.setCursor(0, 1);
  lcd.print("OK!             ");

  positionStepsAzimuth = azimuthAngle2positionSteps(PHYSICAL_LIMIT_AZIMUTH_LEFT);
  positionStepsElevation = elevationAngle2positionSteps(PHYSICAL_LIMIT_ELEVATION_BOTTOM);

  return 0;
}



// Power down the stepper motor for the azimuth drive.
int stepperPowerDownAzimuth() {
  digitalWrite(PIN_MOTOR_AZIMUTH_A_P, LOW);
  digitalWrite(PIN_MOTOR_AZIMUTH_A_N, LOW);
  digitalWrite(PIN_MOTOR_AZIMUTH_B_P, LOW);
  digitalWrite(PIN_MOTOR_AZIMUTH_B_N, LOW);
  return 0;
}



// Power down the stepper motor for the elevation drive.
int stepperPowerDownElevation() {
  digitalWrite(PIN_MOTOR_ELEVATION_A_P, LOW);
  digitalWrite(PIN_MOTOR_ELEVATION_A_N, LOW);
  digitalWrite(PIN_MOTOR_ELEVATION_B_P, LOW);
  digitalWrite(PIN_MOTOR_ELEVATION_B_N, LOW);
  return 0;
}



// Automatically move the telescope between given coordinates for testing.
int autoMoveLoop() {
  int ret = 0;

  #ifdef AUTO_MOVE_LOOP
  int autoMoveCoordinatesNum = sizeof(autoMoveCoordinates) / sizeof(autoMoveCoordinates[0]);
  while (true) {
    for (int i = 0; i < autoMoveCoordinatesNum; i++) {
      ret = moveTelscope(autoMoveCoordinates[i].azimuth, autoMoveCoordinates[i].elevation);
      if (ret) {
        errorHandler(String(CONSOLE_MSG_EOL) + "ERROR: Moving telescope to position " + String(autoMoveCoordinates[i].azimuth, SERIAL_MSG_DECIMALS_AZIMUTH) + stringDegree + ", " + String(autoMoveCoordinates[i].elevation, SERIAL_MSG_DECIMALS_ELEVATION) + stringDegree + " failed because a limit switch was hit! Program stopped!", "ERROR: Move tel.");
      }
    }
  }
  #endif

  return 0;
};



// Initialize the game.
int initGame() {
  int ret;

  // Set up LEDs.
  digitalWrite(PIN_LED_PROGRESS_1, LOW);
  digitalWrite(PIN_LED_PROGRESS_2, LOW);
  digitalWrite(PIN_LED_PROGRESS_3, LOW);
  digitalWrite(PIN_LED_PROGRESS_4, LOW);
  digitalWrite(PIN_LED_PROGRESS_5, LOW);
  digitalWrite(PIN_LED_AZIMUTH_LEFT, LOW);
  digitalWrite(PIN_LED_AZIMUTH_CENTER, LOW);
  digitalWrite(PIN_LED_AZIMUTH_RIGHT, LOW);
  digitalWrite(PIN_LED_ELEVATION_BOTTOM, LOW);
  digitalWrite(PIN_LED_ELEVATION_CENTER, LOW);
  digitalWrite(PIN_LED_ELEVATION_TOP, LOW);
  digitalWrite(PIN_LED_ELEVATION_TOP, LOW);

  // Initialize the hardware before every game.
  #ifdef INIT_HARDWARE_BEFORE_EVERY_GAME
  // Do not initialize the hardware twice at power up or after a reset, i.e. during the first run of the initGame function.
  static bool firstRun = true;
  if (! firstRun) {
    // First move the telescope close to its physical minimum position to minimize the way for finding the zero positions.
    // Don't do this in simulation mode.
    #ifndef SIMULATION_MODE
    ret = moveTelscope(PHYSICAL_LIMIT_AZIMUTH_LEFT + 5, PHYSICAL_LIMIT_ELEVATION_BOTTOM + 5);   // 5 degrees before physical limits.
    #endif
    // Initialize the hardware.
    ret = initHardware();
  }
  firstRun = false;
  #endif

  // Move the telescope to its parking position.
  #ifdef MOVE_TELESCOPE_TO_PARKING_POSITION
  ret = moveTelscope(azimuthPosParking, elevationPosParking);
  if (ret) {
    errorHandler(String(CONSOLE_MSG_EOL) + "ERROR: Moving telescope to position " + String(azimuthPosParking, SERIAL_MSG_DECIMALS_AZIMUTH) + stringDegree + ", " + String(elevationPosParking, SERIAL_MSG_DECIMALS_ELEVATION) + stringDegree + " failed because a limit switch was hit! Program stopped!", "ERROR: Move tel.");
  }
  #endif

  // Power down the stepper motors to save power and keep them cool.
  stepperPowerDownAzimuth();
  stepperPowerDownElevation();

  // Display message.
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));
  SERIAL_CONSOLE.print("MAGIC Game: Push the start button to start a new game.");
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("-= MAGIC Game =-");
  lcd.setCursor(0, 1);
  lcd.print("* Push button! *");

  #ifdef ENABLE_CONTROL_MSG
  SERIAL_CONTROL.print(String(CONTROL_MSG_IDLE) + String(CONTROL_MSG_EOL));
  #endif

  // Wait until the start button is pushed.
  waitStart(0);     // Always wait for the user to push the start button.

  #ifdef ENABLE_CONTROL_MSG
  SERIAL_CONTROL.print(String(CONTROL_MSG_START) + String(CONTROL_MSG_EOL));
  #endif

  // Generate a new gamma position.
  #ifdef USE_FIXED_TARGETS
  fixedTargetSel = random(sizeof(fixedTargets) / sizeof(fixedTargets[0]));
  azimuthTarget = fixedTargets[fixedTargetSel].azimuth;
  elevationTarget = fixedTargets[fixedTargetSel].elevation;
  #else
  azimuthTarget = random(azimuthLimitMin, azimuthLimitMax);
  elevationTarget = random(elevationLimitMin, elevationLimitMax);
  #endif

  #ifdef ENABLE_CONTROL_MSG
  #ifdef USE_FIXED_TARGETS
  SERIAL_CONTROL.print(String(CONTROL_MSG_TARGET) + " " + String(fixedTargetSel + 1) + String(CONTROL_MSG_EOL));
  #else
  SERIAL_CONTROL.print(String(CONTROL_MSG_TARGET) + " " + String(azimuthTarget, SERIAL_MSG_DECIMALS_AZIMUTH) + stringDegree + " " + String(elevationTarget, SERIAL_MSG_DECIMALS_ELEVATION) + stringDegree + String(CONTROL_MSG_EOL));
  #endif
  #endif

  // Display the gamma position.
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "New gamma position: Azimuth: " + String(azimuthTarget, SERIAL_MSG_DECIMALS_AZIMUTH) + stringDegree + ", elevation: " + String(elevationTarget, SERIAL_MSG_DECIMALS_ELEVATION) + stringDegree);
  #ifdef USE_FIXED_TARGETS
  SERIAL_CONSOLE.print(" (fixed target " + String(fixedTargetSel + 1) + ")");
  #endif
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Gamma position:");
  lcd.setCursor(0, 1);
  lcd.print("Az: " + String(int(azimuthTarget)) + ", el: " + String(int(elevationTarget)));
  return 0;
}



// Move the telescope to a given position.
int moveTelscope(float azimuthAngle, float elevationAngle) {
  long stepsAzimuth;
  long stepsElevation;

  // Display message.
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Moving telescope to position: Azimuth: " + String(azimuthAngle, SERIAL_MSG_DECIMALS_AZIMUTH) + stringDegree + ", elevation: " + String(elevationAngle, SERIAL_MSG_DECIMALS_ELEVATION) + stringDegree);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Move telescope: ");
  lcd.setCursor(0, 1);
  lcd.print("Az: " + String(int(azimuthAngle)) + " , el: " + String(int(elevationAngle)));

  // Convert stepper motor positions to azimuth and elevation.
  azimuthActual = positionSteps2azimuthAngle(positionStepsAzimuth);
  elevationActual = positionSteps2elevationAngle(positionStepsElevation);

  // Calculate the stepper motor steps to reach the target position.
  stepsAzimuth = azimuthAngle2positionSteps(azimuthAngle - azimuthActual);
  stepsElevation = elevationAngle2positionSteps(elevationAngle - elevationActual);

  #ifdef MOVE_TELESCOPE_SHOW_MOTOR_STEPS
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "    Motor steps to reach the position: Azimuth: " + String(stepsAzimuth) + ", elevation: " + String(stepsElevation));
  #endif

  // Move the telescope until the target position is reached.
  while ((stepsAzimuth != 0) || (stepsElevation != 0)) {
    // Azimuth: Rotate left.
    if (stepsAzimuth < 0) {
      // Check left limit switch before moving.
      if (! digitalRead(PIN_SW_LIMIT_AZIMUTH_LEFT)) {
        return 1;
      }
      stepperAzimuth.step(-1);      // Move one step left.
      stepsAzimuth++;               // One step less to go.
      positionStepsAzimuth--;       // Update azimuth position.
    // Azimuth: Rotate right.
    } else if (stepsAzimuth > 0) {
      // Check right limit switch before moving.
      if (! digitalRead(PIN_SW_LIMIT_AZIMUTH_RIGHT)) {
        return 2;
      }
      stepperAzimuth.step(1);       // Move one step right.
      stepsAzimuth--;               // One step less to go.
      positionStepsAzimuth++;       // Update azimuth position.
    // Azimuth: No rotation, but insert delay to compensate the missing time.
    } else {
      delay(millisPerStepAzimuth);
    }

    // Elevation: Move down.
    if (stepsElevation < 0) {
      // Check bottom limit switch before moving.
      if (! digitalRead(PIN_SW_LIMIT_ELEVATION_BOTTOM)) {
        return 3;
      }
      stepperElevation.step(-1);  // Move one step down.
      stepsElevation++;           // One step less to go.
      positionStepsElevation--;   // Update elevation position.
    // Elevation: Move up.
    } else if (stepsElevation > 0) {
      // Check top limit switch before moving.
      if (! digitalRead(PIN_SW_LIMIT_ELEVATION_TOP)) {
        return 4;
      }
      stepperElevation.step(1);   // Move one step up.
      stepsElevation--;           // One step less to go.
      positionStepsElevation++;   // Update elevation position.
    // Elevation: No movement, but insert delay to compensate the missing time.
    } else {
      delay(millisPerStepElevation);
    }

    // DEBUG: Show actual azimuth and elevation.
    #ifdef DEBUG_MODE_SHOW_POSITIONS
    // Convert stepper motor positions to azimuth and elevation.
    azimuthActual = positionSteps2azimuthAngle(positionStepsAzimuth);
    elevationActual = positionSteps2elevationAngle(positionStepsElevation);
    #ifdef DEBUG_SERIAL
    SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "DEBUG: Actual azimuth: " + String(azimuthActual, SERIAL_MSG_DECIMALS_AZIMUTH) + stringDegree + ", actual elevation: " + String(elevationActual, SERIAL_MSG_DECIMALS_ELEVATION) + stringDegree);
    #endif
    lcd.setCursor(0, 0);
    lcd.print("DBG act. az: " + String(int(azimuthActual)) + "  ");
    lcd.setCursor(0, 1);
    lcd.print("DBG act. el: " + String(int(elevationActual)) + "  ");
    #endif
  }

  return 0;
}



// Play the game.
int playGame() {
  int ret;
  long stepsAzimuth;
  long stepsElevation;
  long stepsMax;
  long timeStart;
  long timeElapsed;
  float timeElapsedScale;
  #ifdef ENABLE_CONTROL_MSG
  bool controlMsgProgress[6];
  for (int i = 0; i < sizeof(controlMsgProgress) / sizeof(controlMsgProgress[0]); i++) controlMsgProgress[i] = false;
  #ifdef ENABLE_CONTROL_MSG_COUNTDOWN
  int controlMsgCountdown = 10;     // Countdown in seconds.
  #endif
  #ifdef ENABLE_CONTROL_MSG_IN_GAME_EVAL
  #ifdef ENABLE_CONTROL_MSG_IN_GAME_EVAL_DIFF_ONLY
  bool targetMatch = false;
  // Send message for target miss at game start for differential only mode to
  // provide the first feedback.
  SERIAL_CONTROL.print(String(CONTROL_MSG_TARGET_MISS) + String(CONTROL_MSG_EOL));
  #endif
  bool targetMatchAzimuth;
  bool targetMatchElevation;
  #endif
  #endif

  #ifdef DEBUG_MODE_SHOW_STEPS
  lcd.clear();
  #endif

  timeStart = millis();

  #ifdef SIMULATION_MODE
//  timeElapsedScale = 1.0;
  timeElapsedScale = 2.0;
  #else
//  timeElapsedScale = 1.0;
  timeElapsedScale = 2.0;
  #endif

  while (true) {
    // Show the progress of the physics event (gamma ray -> Cherenkov light).
    timeElapsed = millis() - timeStart;
    #ifdef ENABLE_CONTROL_MSG
    #ifdef ENABLE_CONTROL_MSG_PROGRESS
    // Send progress command.
    if ((timeElapsed >   250 * timeElapsedScale) && (controlMsgProgress[0] == false)) { SERIAL_CONTROL.print(String(CONTROL_MSG_PROGRESS) + " 1" + String(CONTROL_MSG_EOL)); controlMsgProgress[0] = true; }
    if ((timeElapsed >  3000 * timeElapsedScale) && (controlMsgProgress[1] == false)) { SERIAL_CONTROL.print(String(CONTROL_MSG_PROGRESS) + " 2" + String(CONTROL_MSG_EOL)); controlMsgProgress[1] = true; }
    if ((timeElapsed >  6000 * timeElapsedScale) && (controlMsgProgress[2] == false)) { SERIAL_CONTROL.print(String(CONTROL_MSG_PROGRESS) + " 3" + String(CONTROL_MSG_EOL)); controlMsgProgress[2] = true; }
    if ((timeElapsed >  9000 * timeElapsedScale) && (controlMsgProgress[3] == false)) { SERIAL_CONTROL.print(String(CONTROL_MSG_PROGRESS) + " 4" + String(CONTROL_MSG_EOL)); controlMsgProgress[3] = true; }
    if ((timeElapsed > 12000 * timeElapsedScale) && (controlMsgProgress[4] == false)) { SERIAL_CONTROL.print(String(CONTROL_MSG_PROGRESS) + " 5" + String(CONTROL_MSG_EOL)); controlMsgProgress[4] = true; }
    if ((timeElapsed > 15000 * timeElapsedScale) && (controlMsgProgress[5] == false)) { SERIAL_CONTROL.print(String(CONTROL_MSG_PROGRESS) + " 6" + String(CONTROL_MSG_EOL)); controlMsgProgress[5] = true; }
    #endif
    #ifdef ENABLE_CONTROL_MSG_COUNTDOWN
    // Send countdown command.
    if (timeElapsed > (15000 * timeElapsedScale) - (controlMsgCountdown * 1000)) { SERIAL_CONTROL.print(String(CONTROL_MSG_COUNTDOWN) + " " + String(controlMsgCountdown) + String(CONTROL_MSG_EOL)); controlMsgCountdown -= 1; }
    #endif
    #endif
    if (timeElapsed >   250 * timeElapsedScale) digitalWrite(PIN_LED_PROGRESS_1, HIGH);
    if (timeElapsed >  3000 * timeElapsedScale) digitalWrite(PIN_LED_PROGRESS_2, HIGH);
    if (timeElapsed >  6000 * timeElapsedScale) digitalWrite(PIN_LED_PROGRESS_3, HIGH);
    if (timeElapsed >  9000 * timeElapsedScale) digitalWrite(PIN_LED_PROGRESS_4, HIGH);
    if (timeElapsed > 12000 * timeElapsedScale) digitalWrite(PIN_LED_PROGRESS_5, HIGH);
    #ifndef INFINITE_GAME_LOOP
    if (timeElapsed > 15000 * timeElapsedScale) {
      // End the game.
      ret = endGame();
      return ret;
    }
    #endif

    // Get the values from the analog joystick.
    #ifdef JOYSTICK_INVERT_AZIMUTH
    stepsAzimuth = analog2steps(0x3ff - analogRead(PIN_JOYSTICK_AZ));
    #else
    stepsAzimuth = analog2steps(analogRead(PIN_JOYSTICK_AZ));
    #endif
    #ifdef JOYSTICK_INVERT_ELEVATION
    stepsElevation = analog2steps(0x3ff - analogRead(PIN_JOYSTICK_EL));
    #else
    stepsElevation = analog2steps(analogRead(PIN_JOYSTICK_EL));
    #endif

    // DEBUG: Show steps for azimuth and elevation.
    #ifdef DEBUG_MODE_SHOW_STEPS
    #ifdef DEBUG_SERIAL
    SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "DEBUG: Steps azimuth: " + String(stepsAzimuth) + ", steps elevation: " + String(stepsElevation));
    #endif
    lcd.setCursor(0, 0);
    lcd.print("DBG stp. az: " + String(stepsAzimuth) + "  ");
    lcd.setCursor(0, 1);
    lcd.print("DBG stp. el: " + String(stepsElevation) + "  ");
    #endif

    // Check if any limit switch got activated.
    if (!digitalRead(PIN_SW_LIMIT_AZIMUTH_LEFT) || !digitalRead(PIN_SW_LIMIT_AZIMUTH_RIGHT) ||
        !digitalRead(PIN_SW_LIMIT_ELEVATION_BOTTOM) || !digitalRead(PIN_SW_LIMIT_ELEVATION_TOP)
    ) {
      errorHandler(String(CONSOLE_MSG_EOL) + "ERROR: Limit switch hit while moving the telescope during the game! Program stopped!", "ERROR: Limit sw.");
    }

    // Move the stepper motors.
    // Move both motors in parallel step by step.
    stepsMax = max(abs(analog2steps(0)), abs(analog2steps(1024)));  // Get the maximum number of possible steps.
    for (int i = 0; i < stepsMax; i++) {
      // Convert stepper motor positions to azimuth and elevation.
      azimuthActual = positionSteps2azimuthAngle(positionStepsAzimuth);
      elevationActual = positionSteps2elevationAngle(positionStepsElevation);

      // Azimuth: Rotate left.
      #ifdef IGNORE_SOFT_LIMITS_AZIMUTH
      if ((stepsAzimuth < 0) && (digitalRead(PIN_SW_LIMIT_AZIMUTH_LEFT))) {
      #else
      if ((stepsAzimuth < 0) && (azimuthActual > azimuthLimitMin) && (digitalRead(PIN_SW_LIMIT_AZIMUTH_LEFT))) {
      #endif
        stepperAzimuth.step(-1);    // Move one step left.
        stepsAzimuth++;             // One step less to go.
        positionStepsAzimuth--;     // Update azimuth position.
      // Azimuth: Rotate right.
      #ifdef IGNORE_SOFT_LIMITS_AZIMUTH
      } else if ((stepsAzimuth > 0) && (digitalRead(PIN_SW_LIMIT_AZIMUTH_RIGHT))) {
      #else
      } else if ((stepsAzimuth > 0) && (azimuthActual < azimuthLimitMax) && (digitalRead(PIN_SW_LIMIT_AZIMUTH_RIGHT))) {
      #endif
        stepperAzimuth.step(1);     // Move one step right.
        stepsAzimuth--;             // One step less to go.
        positionStepsAzimuth++;     // Update azimuth position.
      // Azimuth: No rotation, but insert delay to compensate the missing time.
      } else {
        delay(millisPerStepAzimuth);
      }

      // Elevation: Move down.
      #ifdef IGNORE_SOFT_LIMITS_ELEVATION
      if ((stepsElevation < 0) && (digitalRead(PIN_SW_LIMIT_ELEVATION_BOTTOM))) {
      #else
      if ((stepsElevation < 0) && (elevationActual > elevationLimitMin) && (digitalRead(PIN_SW_LIMIT_ELEVATION_BOTTOM))) {
      #endif
        stepperElevation.step(-1);  // Move one step down.
        stepsElevation++;           // One step less to go.
        positionStepsElevation--;   // Update elevation position.
      // Elevation: Move up.
      #ifdef IGNORE_SOFT_LIMITS_ELEVATION
      } else if ((stepsElevation > 0) && (digitalRead(PIN_SW_LIMIT_ELEVATION_TOP))) {
      #else
      } else if ((stepsElevation > 0) && (elevationActual < elevationLimitMax) && (digitalRead(PIN_SW_LIMIT_ELEVATION_TOP))) {
      #endif
        stepperElevation.step(1);   // Move one step up.
        stepsElevation--;           // One step less to go.
        positionStepsElevation++;   // Update elevation position.
      // Elevation: No movement, but insert delay to compensate the missing time.
      } else {
        delay(millisPerStepElevation);
      }
    }

    // Set the LEDs for the azimuth and elevation according to the current position.
    // If enabled, also send the command for matching or missing the target.
    // Azimuth position too far left -> move right!
    #ifdef ENABLE_CONTROL_MSG
    #ifdef ENABLE_CONTROL_MSG_IN_GAME_EVAL
    targetMatchAzimuth = false;
    targetMatchElevation = false;
    #endif
    #endif
    if (azimuthActual < azimuthTarget - azimuthTolerance) {
      digitalWrite(PIN_LED_AZIMUTH_LEFT, LOW);
      digitalWrite(PIN_LED_AZIMUTH_CENTER, LOW);
      digitalWrite(PIN_LED_AZIMUTH_RIGHT, HIGH);
    // Azimuth position too far right -> move left!
    } else if (azimuthActual > azimuthTarget + azimuthTolerance) {
      digitalWrite(PIN_LED_AZIMUTH_LEFT, HIGH);
      digitalWrite(PIN_LED_AZIMUTH_CENTER, LOW);
      digitalWrite(PIN_LED_AZIMUTH_RIGHT, LOW);
    // Azimuth position on the spot!
    } else {
      digitalWrite(PIN_LED_AZIMUTH_LEFT, LOW);
      digitalWrite(PIN_LED_AZIMUTH_CENTER, HIGH);
      digitalWrite(PIN_LED_AZIMUTH_RIGHT, LOW);
      #ifdef ENABLE_CONTROL_MSG
      #ifdef ENABLE_CONTROL_MSG_IN_GAME_EVAL
      targetMatchAzimuth = true;
      #endif
      #endif
    }
    // Elevation too low -> move up!
    if (elevationActual < elevationTarget - elevationTolerance) {
      digitalWrite(PIN_LED_ELEVATION_BOTTOM, LOW);
      digitalWrite(PIN_LED_ELEVATION_CENTER, LOW);
      digitalWrite(PIN_LED_ELEVATION_TOP, HIGH);
    // Elevation too high -> move down!
    } else if (elevationActual > elevationTarget + elevationTolerance) {
      digitalWrite(PIN_LED_ELEVATION_BOTTOM, HIGH);
      digitalWrite(PIN_LED_ELEVATION_CENTER, LOW);
      digitalWrite(PIN_LED_ELEVATION_TOP, LOW);
    // Elevation on the spot!
    } else {
      digitalWrite(PIN_LED_ELEVATION_BOTTOM, LOW);
      digitalWrite(PIN_LED_ELEVATION_CENTER, HIGH);
      digitalWrite(PIN_LED_ELEVATION_TOP, LOW);
      #ifdef ENABLE_CONTROL_MSG
      #ifdef ENABLE_CONTROL_MSG_IN_GAME_EVAL
      targetMatchElevation = true;
      #endif
      #endif
    }

    // DEBUG: Show actual azimuth and elevation.
    #ifdef DEBUG_MODE_SHOW_POSITIONS
    #ifdef DEBUG_SERIAL
    SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "DEBUG: Actual azimuth: " + String(azimuthActual, SERIAL_MSG_DECIMALS_AZIMUTH) + stringDegree + ", actual elevation: " + String(elevationActual, SERIAL_MSG_DECIMALS_ELEVATION) + stringDegree);
    #endif
    lcd.setCursor(0, 0);
    lcd.print("DBG act. az: " + String(int(azimuthActual)) + "  ");
    lcd.setCursor(0, 1);
    lcd.print("DBG act. el: " + String(int(elevationActual)) + "  ");
    #endif

    #ifdef ENABLE_CONTROL_MSG
    #ifdef ENABLE_CONTROL_MSG_IN_GAME_EVAL
    #ifdef ENABLE_CONTROL_MSG_IN_GAME_EVAL_DIFF_ONLY
      if (targetMatchAzimuth && targetMatchElevation) {
        if (targetMatch == false) {
          SERIAL_CONTROL.print(String(CONTROL_MSG_TARGET_MATCH) + String(CONTROL_MSG_EOL));
          targetMatch = true;
          #ifdef END_GAME_AT_TARGET_POSITION
          // End the game.
          ret = endGame();
          return ret;
          #endif
        }
      } else {
        if (targetMatch == true) {
          SERIAL_CONTROL.print(String(CONTROL_MSG_TARGET_MISS) + String(CONTROL_MSG_EOL));
          targetMatch = false;
        }
      }
    #else
    if (targetMatchAzimuth && targetMatchElevation) {
      SERIAL_CONTROL.print(String(CONTROL_MSG_TARGET_MATCH) + String(CONTROL_MSG_EOL));
      #ifdef END_GAME_AT_TARGET_POSITION
      // End the game.
      ret = endGame();
      return ret;
      #endif
    } else {
      SERIAL_CONTROL.print(String(CONTROL_MSG_TARGET_MISS) + String(CONTROL_MSG_EOL));
    }
    #endif
    #endif
    #endif

    #ifdef END_GAME_AT_TARGET_POSITION
    if ((azimuthActual > azimuthTarget - azimuthTolerance) && (azimuthActual < azimuthTarget + azimuthTolerance) &&
        (elevationActual > elevationTarget - elevationTolerance) && (elevationActual < elevationTarget + elevationTolerance))
    {
      // End the game.
      ret = endGame();
      return ret;
    }
    #endif
  }
  return 0;
}



// End the game.
int endGame() {
  int ret;

  // Power down the stepper motors to save power and keep them cool.
  stepperPowerDownAzimuth();
  stepperPowerDownElevation();

  // Evaluate the game result.
  ret = evalGameResult();

  return ret;
}



// Convert an analog value from the joystick (potentiometers) into stepper motor steps.
long analog2steps(int analog) {
  long steps = 0;
  if (analog < 200)       steps = -4;
  else if (analog < 300)  steps = -2;
  else if (analog < 400)  steps = -1;
  else if (analog < 624)  steps =  0;
  else if (analog < 724)  steps = +1;
  else if (analog < 824)  steps = +2;
  else                    steps = +4;
  return steps;
}



// Convert the stepper motor position in steps to the azimuth angle in degrees.
float positionSteps2azimuthAngle(long steps) {
  return ((float) steps / STEPS_AZIMUTH) * AZIMUTH_DEGREES_PER_REVOLUTION;
}



// Convert the stepper motor position in steps to the elevation angle in degrees.
float positionSteps2elevationAngle(long steps) {
  return ((float) steps / STEPS_ELEVATION) * ELEVATION_DEGREES_PER_REVOLUTION;
}



// Convert the azimuth angle in degrees to the stepper motor position in steps.
long azimuthAngle2positionSteps(float azimuthAngle) {
  return (long) ((azimuthAngle / AZIMUTH_DEGREES_PER_REVOLUTION) * STEPS_AZIMUTH);
}



// Convert the elevation angle in degrees to the stepper motor position in steps.
long elevationAngle2positionSteps(float elevationAngle) {
  return (long) ((elevationAngle / ELEVATION_DEGREES_PER_REVOLUTION) * STEPS_ELEVATION);
}



// Evaluate the result of the game.
int evalGameResult() {
  int ret = 0;

  // Display evaluation.
  lcd.clear();
  if ((azimuthActual > azimuthTarget - azimuthTolerance) && (azimuthActual < azimuthTarget + azimuthTolerance) &&
      (elevationActual > elevationTarget - elevationTolerance) && (elevationActual < elevationTarget + elevationTolerance)
  ) {
    #ifdef ENABLE_CONTROL_MSG
    SERIAL_CONTROL.print(String(CONTROL_MSG_SUCCESS) + String(CONTROL_MSG_EOL));
    #endif
    SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));
    SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "**************************************");
    SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Congratulations! You caught the gamma!");
    SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "**************************************");
    lcd.setCursor(0, 0);
    lcd.print("Congratulations!");
    lcd.setCursor(0, 1);
    lcd.print("Gamma caught! :)");
    ret = 0;
  } else {
    #ifdef ENABLE_CONTROL_MSG
    SERIAL_CONTROL.print(String(CONTROL_MSG_FAILURE) + String(CONTROL_MSG_EOL));
    #endif
    SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));
    SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "****************************");
    SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Sorry, You missed the gamma!");
    SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "****************************");
    lcd.setCursor(0, 0);
    lcd.print("Sorry. :(");
    lcd.setCursor(0, 1);
    lcd.print("Gamma missed.");
    ret = 1;
  }
  // Print some extra information.
  azimuthActual = positionSteps2azimuthAngle(positionStepsAzimuth);
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Game Details");
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "============");
  #ifdef USE_FIXED_TARGETS
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Fixed target " + String(fixedTargetSel + 1) + ".");
  #else
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Random target.");
  #endif
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Actual azimuth: " + String(azimuthActual, SERIAL_MSG_DECIMALS_AZIMUTH) + stringDegree);
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Target azimuth: " + String(azimuthTarget, SERIAL_MSG_DECIMALS_AZIMUTH) + stringDegree);
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Tolerance for azimuth: +/-" + String(azimuthTolerance, SERIAL_MSG_DECIMALS_AZIMUTH) + stringDegree);
  elevationActual = positionSteps2elevationAngle(positionStepsElevation);
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Actual elevation: " + String(elevationActual, SERIAL_MSG_DECIMALS_ELEVATION) + stringDegree);
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Target elevation: " + String(elevationTarget, SERIAL_MSG_DECIMALS_ELEVATION) + stringDegree);
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Tolerance for elevation: +/-" + String(elevationTolerance, SERIAL_MSG_DECIMALS_ELEVATION) + stringDegree);
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));

  // Wait until the start button is pushed. Alternatively use a timeout.
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL) + "Please press the start button to continue (timeout in " + String(TIMEOUT_PREPARE_NEW_GAME / 1000) + " seconds).");
  SERIAL_CONSOLE.print(String(CONSOLE_MSG_EOL));
  waitStart(TIMEOUT_PREPARE_NEW_GAME);
  delay(1000);

  return ret;
}
