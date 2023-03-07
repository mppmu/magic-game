// File: MagicGame.ino
// Auth: M. Fras, Electronics Division, MPI for Physics, Munich
// Mod.: M. Fras, Electronics Division, MPI for Physics, Munich
// Date: 25 Nov 2022
// Rev.: 07 Mar 2023
//
// Firmware for the Arduino Mega 2560 Rev 3 to control the telescope model of
// the MAGIC Game via the MAGIC Game board.
//
// Stepper motors:
// - Azimuth: 28BYJ-48 with 1/64 gearing
// - Elevation: 28BYJ-48 with 1/25 gearing (transmission from TS-24BYJ48A-25)
// - Directions:
//   - Azimuth left:    -1 step
//   - Azimuth right:   +1 step
//   - Elevation down:  -1 step
//   - Elevation up:    +1 step
//



#include <Stepper.h>
#include <LiquidCrystal.h>



#define FW_NAME         "MagicGame"
#define FW_VERSION      "0.0.6"
#define FW_RELEASEDATE  "07 Mar 2023"



// For simulation in SimulIDE.
#define SIMULATION_MODE

// For debugging.
//#define DEBUG_MODE_SHOW_STEPS
#define DEBUG_MODE_SHOW_POSITIONS

// Debugging over the serial interface.
#define DEBUG_SERIAL

// Define serial interfaces.
#define SERIAL_CONSOLE                      Serial
#define SERIAL_CONTROL                      Serial1

// Define control messages for commuincation with the exhibition booth.
#define ENABLE_CONTROL_MSG
#define CONTROL_MSG_ERROR                   "ERROR"
#define CONTROL_MSG_BOOT                    "BOOT"
#define CONTROL_MSG_INIT                    "INIT"
#define CONTROL_MSG_IDLE                    "IDLE"
#define CONTROL_MSG_START                   "START"
#define CONTROL_MSG_TARGET                  "TARGET"
#define CONTROL_MSG_PROGRESS                "PROGRESS"
#define CONTROL_MSG_SUCCESS                 "SUCCESS"
#define CONTROL_MSG_FAILURE                 "FAILURE"

// Define number of decimals in serial messages.
#define SERIAL_MSG_DECIMALS_AZIMUTH         1
#define SERIAL_MSG_DECIMALS_ELEVATION       1

// Ignore soft limits when moving the telescope.
//#define IGNORE_SOFT_LIMITS_AZIMUTH
//#define IGNORE_SOFT_LIMITS_ELEVATION

// Move the telescope to its parking position before starting the game.
#define MOVE_TELESCOPE_TO_PARKING_POSITION

// Define pins.
#define PIN_JOYSTICK_AZ                     A0
#define PIN_JOYSTICK_EL                     A1
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
#define AZIMUTH_DEGREES_PER_REVOLUTION      360     // Degress of azimuth per stepper motor revolution.
#define AZIMUTH_REVERSE_DIRECTION
#define STEPS_ELEVATION                     64      // Steps per revolution.
#define SPEED_ELEVATION                     10      // RPM.
#define ELEVATION_DEGREES_PER_REVOLUTION    90      // Degress of elevation per stepper motor revolution.
#define ELEVATION_REVERSE_DIRECTION
#else
// Stepper motor for azimuth:
// - Stride angle: 5.625°/64
// - Steps per revolution: 4096
#define STEPS_AZIMUTH                       4096    // Steps per revolution.
#define SPEED_AZIMUTH                       1.2     // RPM.
#define AZIMUTH_DEGREES_PER_REVOLUTION      360     // Degress of azimuth per stepper motor revolution.
#define AZIMUTH_REVERSE_DIRECTION
// Stepper motor for elevation:
// - Stride angle: 5.625°/25
// - Steps per revolution: 1600
#define STEPS_ELEVATION                     1600    // Steps per revolution.
#define SPEED_ELEVATION                     10      // RPM.
#define ELEVATION_DEGREES_PER_REVOLUTION    12.4    // Degress of elevation per stepper motor revolution.
#define ELEVATION_REVERSE_DIRECTION
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

// Create an instance of the LCD.
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

// Azimuth and elevation software limits.
#ifdef SIMULATION_MODE
const float azimuthLimitMin     = 10.0;
const float azimuthLimitMax     = 350.0;
const float elevationLimitMin   = 10.0;
const float elevationLimitMax   = 80.0;
#else
const float azimuthLimitMin     = 10.0;
//const float azimuthLimitMax     = 270.0;
const float azimuthLimitMax     = 90.0;
const float elevationLimitMin   = 10.0;
//const float elevationLimitMax   = 110.0;
const float elevationLimitMax   = 45.0;
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
float azimuthActual             = 0.0;
float azimuthTarget             = 0.0;
const float azimuthTolerance    = 20.0;
float elevationActual           = 0.0;
float elevationTarget           = 0.0;
const float elevationTolerance  = 10.0;



void setup() {
  // Initialize the serial interfaces.
  SERIAL_CONSOLE.begin(9600);       // Console interface for debugging.
  SERIAL_CONTROL.begin(9600);       // Control interface for communication with exhibition booth.
  // Send control message.
  #ifdef ENABLE_CONTROL_MSG
  SERIAL_CONTROL.print("\r\n");
  SERIAL_CONTROL.print(String(CONTROL_MSG_BOOT) + "\r\n");
  #endif
  // Print a message on the serial console.
  SERIAL_CONSOLE.print("\r\n*** Welcome to the MAGIC Game: Catch the Gamma! ***");
  SERIAL_CONSOLE.print("\r\n");
  SERIAL_CONSOLE.print("\r\nFirmware info:");
  SERIAL_CONSOLE.print("\r\n- Name: ");
  SERIAL_CONSOLE.print(FW_NAME);
  SERIAL_CONSOLE.print("\r\n- Ver.: ");
  SERIAL_CONSOLE.print(FW_VERSION);
  SERIAL_CONSOLE.print("\r\n- Date: ");
  SERIAL_CONSOLE.print(FW_RELEASEDATE);
  SERIAL_CONSOLE.print("\r\n");

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

  // Wait until the start button is pushed.
  SERIAL_CONSOLE.print("\r\nPlease press the start button to continue.");
  waitStart();
  delay(1000);
}



void loop() {
  initHardware();
  while (true) {
    // Power down the stepper motors to save power and keep them cool.
    stepperPowerDownAzimuth();
    stepperPowerDownElevation();
    // Initialize the game.
    initGame();
    // Play the game.
    playGame();
  }
}



// Wait until the start button is pushed.
int waitStart() {
  while (digitalRead(PIN_BUTTON_START) && digitalRead(PIN_JOYSTICK_BUTTON));
  return 0;
}



// Initialize the hardware.
int initHardware() {
  int ret = 0;

  #ifdef ENABLE_CONTROL_MSG
  SERIAL_CONTROL.print(String(CONTROL_MSG_INIT) + "\r\n");
  #endif

  #ifdef SIMULATION_MODE
  // Do not try to find the zero positions of the stepper motors in simulation mode.
  ret = 0;
  #else
  // Find the zero positions of the stepper motors.
//  ret = stepperFindZeroPosition();
  #endif
  // Error while finding the zero positions.
  // => Halt the execution and display an error message.
  if (ret) {
    digitalWrite(PIN_LED_OK, LOW);
    digitalWrite(PIN_LED_ERROR, HIGH);
    SERIAL_CONSOLE.print("\r\nERROR: Zero position of stepper motors not found! Program stopped!");
    SERIAL_CONSOLE.print("\r\nPress the reset button to reboot.");
    SERIAL_CONSOLE.print("\r\n");
    lcd.clear();
    while (true) {
      #ifdef ENABLE_CONTROL_MSG
      SERIAL_CONTROL.print(String(CONTROL_MSG_ERROR) + "\r\n");
      #endif
      lcd.setCursor(0, 0);
      lcd.print("ERROR: Zero pos.");
      lcd.setCursor(0, 1);
      lcd.print("PROGRAM STOPPED!");
      delay(1000);
      lcd.setCursor(0, 1);
      lcd.print("PRESS RESET!    ");
      delay(1000);
    }
  }

  digitalWrite(PIN_LED_OK, HIGH);

  return 0;
}



// Drive the stepper motors to their limit switch in order to get the zero positions.
int stepperFindZeroPosition() {
  long steps;
  // Display message.
  SERIAL_CONSOLE.print("\r\nFinding the zero positions of the stepper motors.");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Find zero pos.: ");
  // Azimuth: Move to left until limit switch gets activated.
  SERIAL_CONSOLE.print("\r\n- Azimuth... ");
  lcd.setCursor(0, 1);
  lcd.print("Azimuth...      ");
  steps = 0;
  while (digitalRead(PIN_SW_LIMIT_AZIMUTH_LEFT)) {
    stepperAzimuth.step(-1);    // Move one step left.
    steps++;
    // Error: One complete rotation without limit switch getting activated!
    if (steps > (360 / AZIMUTH_DEGREES_PER_REVOLUTION) * STEPS_AZIMUTH) {
      SERIAL_CONSOLE.print("FAILED!");
      lcd.setCursor(0, 1);
      lcd.print("Azimuth FALIED! ");
      return 1;
    }
  }
  SERIAL_CONSOLE.print("OK.");
  // Elevation: Move down until limit switch gets activated.
  SERIAL_CONSOLE.print("\r\n- Elevation... ");
  lcd.setCursor(0, 1);
  lcd.print("Elevation...    ");
  steps = 0;
  while (digitalRead(PIN_SW_LIMIT_ELEVATION_BOTTOM)) {
    stepperElevation.step(-1);  // Move one step down.
    steps++;
    // Error: Half a rotation without limit switch getting activated!
    if (steps > (180 / AZIMUTH_DEGREES_PER_REVOLUTION) * STEPS_ELEVATION) {
      SERIAL_CONSOLE.print("FAILED!");
      lcd.setCursor(0, 1);
      lcd.print("Elevation FAILED");
      return 2;
    }
  }
  SERIAL_CONSOLE.print("OK.");

  lcd.setCursor(0, 1);
  lcd.print("OK!             ");

  positionStepsAzimuth = 0;
  positionStepsElevation = 0;

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

  // Move the telescope to its parking position.
  #ifdef MOVE_TELESCOPE_TO_PARKING_POSITION
  ret = moveTelscope(azimuthPosParking, elevationPosParking);
  if (ret) {
    digitalWrite(PIN_LED_OK, LOW);
    digitalWrite(PIN_LED_ERROR, HIGH);
    SERIAL_CONSOLE.print("\r\nERROR: Moving telescope to position " + String(azimuthPosParking, SERIAL_MSG_DECIMALS_AZIMUTH) + ", " + String(elevationPosParking, SERIAL_MSG_DECIMALS_ELEVATION) + " failed! Program stopped!");
    SERIAL_CONSOLE.print("\r\nPress the reset button to reboot.");
    SERIAL_CONSOLE.print("\r\n");
    lcd.clear();
    while (true) {
      #ifdef ENABLE_CONTROL_MSG
      SERIAL_CONTROL.print(String(CONTROL_MSG_ERROR) + "\r\n");
      #endif
      lcd.setCursor(0, 0);
      lcd.print("ERROR: Move tel.");
      lcd.setCursor(0, 1);
      lcd.print("PROGRAM STOPPED!");
      delay(1000);
      lcd.setCursor(0, 1);
      lcd.print("PRESS RESET!    ");
      delay(1000);
    }
  }
  #endif

  // Display message.
  SERIAL_CONSOLE.print("\r\n\r\nMAGIC Game: Push the start button to start a new game.\r\n");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("-= MAGIC Game =-");
  lcd.setCursor(0, 1);
  lcd.print("* Push button! *");

  #ifdef ENABLE_CONTROL_MSG
  SERIAL_CONTROL.print(String(CONTROL_MSG_IDLE) + "\r\n");
  #endif

  // Wait until the start button is pushed.
  waitStart();

  #ifdef ENABLE_CONTROL_MSG
  SERIAL_CONTROL.print(String(CONTROL_MSG_START) + "\r\n");
  #endif

  // Generate a new gamma position.
  azimuthTarget = random(azimuthLimitMin, azimuthLimitMax);
  elevationTarget = random(elevationLimitMin, elevationLimitMax);

  #ifdef ENABLE_CONTROL_MSG
  SERIAL_CONTROL.print(String(CONTROL_MSG_TARGET) + " " + String(azimuthTarget, SERIAL_MSG_DECIMALS_AZIMUTH) + " " + String(elevationTarget, SERIAL_MSG_DECIMALS_ELEVATION) + "\r\n");
  #endif

  // Display the gamma position.
  SERIAL_CONSOLE.print("\r\nNew gamma position: Azimuth: " + String(azimuthTarget, SERIAL_MSG_DECIMALS_AZIMUTH) + ", elevation: " + String(elevationTarget, SERIAL_MSG_DECIMALS_ELEVATION));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Gamma position:");
  lcd.setCursor(0, 1);
  lcd.print("Az: " + String(int(azimuthTarget)) + " , el: " + String(int(elevationTarget)));
  return 0;
}



// Move the telescope to a given position.
int moveTelscope(float azimuthAngle, float elevationAngle) {
  long stepsAzimuth;
  long stepsElevation;

  // Display message.
  SERIAL_CONSOLE.print("\r\nMoving telescope to position: Azimuth: " + String(azimuthAngle, SERIAL_MSG_DECIMALS_AZIMUTH) + ", elevation: " + String(elevationAngle, SERIAL_MSG_DECIMALS_ELEVATION));
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
    SERIAL_CONSOLE.print("\r\nDEBUG: Actual azimuth: " + String(azimuthActual, SERIAL_MSG_DECIMALS_AZIMUTH) + ", actual elevation: " + String(elevationActual, SERIAL_MSG_DECIMALS_ELEVATION));
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
  for (int i = 0; i < sizeof(controlMsgProgress) / sizeof(bool); i++) controlMsgProgress[i] = false;
  #endif

  #ifdef DEBUG_MODE_SHOW_STEPS
  lcd.clear();
  #endif

  timeStart = millis();

  while (true) {
    // Show the progress of the physics event (gamma ray -> Cherenkov light).
    timeElapsed = millis() - timeStart;
    #ifdef SIMULATION_MODE
    timeElapsedScale = 4.0;
    #else
//    timeElapsedScale = 2.0;
    timeElapsedScale = 4.0;
    #endif
    #ifdef ENABLE_CONTROL_MSG
    if ((timeElapsed > 250  * timeElapsedScale) && (controlMsgProgress[0] == false)) { SERIAL_CONTROL.print(String(CONTROL_MSG_PROGRESS) + " 1" + "\r\n"); controlMsgProgress[0] = true; }
    if ((timeElapsed > 1000 * timeElapsedScale) && (controlMsgProgress[1] == false)) { SERIAL_CONTROL.print(String(CONTROL_MSG_PROGRESS) + " 2" + "\r\n"); controlMsgProgress[1] = true; }
    if ((timeElapsed > 2000 * timeElapsedScale) && (controlMsgProgress[2] == false)) { SERIAL_CONTROL.print(String(CONTROL_MSG_PROGRESS) + " 3" + "\r\n"); controlMsgProgress[2] = true; }
    if ((timeElapsed > 3000 * timeElapsedScale) && (controlMsgProgress[3] == false)) { SERIAL_CONTROL.print(String(CONTROL_MSG_PROGRESS) + " 4" + "\r\n"); controlMsgProgress[3] = true; }
    if ((timeElapsed > 4000 * timeElapsedScale) && (controlMsgProgress[4] == false)) { SERIAL_CONTROL.print(String(CONTROL_MSG_PROGRESS) + " 5" + "\r\n"); controlMsgProgress[4] = true; }
    if ((timeElapsed > 5000 * timeElapsedScale) && (controlMsgProgress[5] == false)) { SERIAL_CONTROL.print(String(CONTROL_MSG_PROGRESS) + " 6" + "\r\n"); controlMsgProgress[5] = true; }
    #endif
    if (timeElapsed > 250  * timeElapsedScale) digitalWrite(PIN_LED_PROGRESS_1, HIGH);
    if (timeElapsed > 1000 * timeElapsedScale) digitalWrite(PIN_LED_PROGRESS_2, HIGH);
    if (timeElapsed > 2000 * timeElapsedScale) digitalWrite(PIN_LED_PROGRESS_3, HIGH);
    if (timeElapsed > 3000 * timeElapsedScale) digitalWrite(PIN_LED_PROGRESS_4, HIGH);
    if (timeElapsed > 4000 * timeElapsedScale) digitalWrite(PIN_LED_PROGRESS_5, HIGH);
//    if (timeElapsed > 5000 * timeElapsedScale) {
//      Power down the stepper motors to save power and keep them cool.
//      stepperPowerDownAzimuth();
//      stepperPowerDownElevation();
//      ret = evalGameResult();
//      return ret;
//    }

    // Get the values from the analog joystick.
    stepsAzimuth = analog2steps(analogRead(PIN_JOYSTICK_AZ));
    stepsElevation = analog2steps(analogRead(PIN_JOYSTICK_EL));

    // DEBUG: Show steps for azimuth and elevation.
    #ifdef DEBUG_MODE_SHOW_STEPS
    #ifdef DEBUG_SERIAL
    SERIAL_CONSOLE.print("\r\nDEBUG: Steps azimuth: " + String(stepsAzimuth) + ", steps elevation: " + String(stepsElevation));
    #endif
    lcd.setCursor(0, 0);
    lcd.print("DBG stp. az: " + String(stepsAzimuth) + "  ");
    lcd.setCursor(0, 1);
    lcd.print("DBG stp. el: " + String(stepsElevation) + "  ");
    #endif

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
    // Azimuth position too far left -> move right!
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
    }

    // DEBUG: Show actual azimuth and elevation.
    #ifdef DEBUG_MODE_SHOW_POSITIONS
    #ifdef DEBUG_SERIAL
    SERIAL_CONSOLE.print("\r\nDEBUG: Actual azimuth: " + String(azimuthActual, SERIAL_MSG_DECIMALS_AZIMUTH) + ", actual elevation: " + String(elevationActual, SERIAL_MSG_DECIMALS_ELEVATION));
    #endif
    lcd.setCursor(0, 0);
    lcd.print("DBG act. az: " + String(int(azimuthActual)) + "  ");
    lcd.setCursor(0, 1);
    lcd.print("DBG act. el: " + String(int(elevationActual)) + "  ");
    #endif
  }
  return 0;
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
  int ret;

  // Display evaluation.
  lcd.clear();
  if ((azimuthActual > azimuthTarget - azimuthTolerance) && (azimuthActual < azimuthTarget + azimuthTolerance) &&
      (elevationActual > elevationTarget - elevationTolerance) && (elevationActual < elevationTarget + elevationTolerance)
  ) {
    #ifdef ENABLE_CONTROL_MSG
    SERIAL_CONTROL.print(String(CONTROL_MSG_SUCCESS) + "\r\n");
    #endif
    SERIAL_CONSOLE.print("\r\nCongratulations! You caught the gamma!");
    lcd.setCursor(0, 0);
    lcd.print("Congratulations!");
    lcd.setCursor(0, 1);
    lcd.print("Gamma caught!");
    ret = 0;
  } else {
    #ifdef ENABLE_CONTROL_MSG
    SERIAL_CONTROL.print(String(CONTROL_MSG_FAILURE) + "\r\n");
    #endif
    SERIAL_CONSOLE.print("\r\nSorry, You missed the gamma!");
    lcd.setCursor(0, 0);
    lcd.print("Sorry. :(");
    lcd.setCursor(0, 1);
    lcd.print("Gamma missed.");
    ret = 1;
  }

  // Wait until the start button is pushed.
  waitStart();
  delay(1000);

  return ret;
}
