// File: MagicGame.ino
// Auth: M. Fras, Electronics Division, MPI for Physics, Munich
// Mod.: M. Fras, Electronics Division, MPI for Physics, Munich
// Date: 25 Nov 2022
// Rev.: 24 Jan 2023
//



#include <Stepper.h>
#include <LiquidCrystal.h>



#define FW_NAME         "MagicGame"
#define FW_VERSION      "0.0.2"
#define FW_RELEASEDATE  "24 Jan 2023"



// For simulation in SimulIDE.
#define SIMULATION_MODE

// For debugging.
//#define DEBUG_MODE_0
#define DEBUG_MODE_1

// Debugging over the serial interface.
#define DEBUG_SERIAL

// Define pins.
#define PIN_JOYSTICK_AZ                 A0
#define PIN_JOYSTICK_EL                 A1
#define PIN_JOYSTICK_BUTTON             A2
#define PIN_LED_PROGRESS_1              2
#define PIN_LED_PROGRESS_2              3
#define PIN_LED_PROGRESS_3              4
#define PIN_LED_PROGRESS_4              5
#define PIN_LED_PROGRESS_5              6
#define PIN_SW_LIMIT_AZIMUTH_LEFT       22
#define PIN_SW_LIMIT_AZIMUTH_RIGHT      23
#define PIN_SW_LIMIT_ELEVATION_BOTTOM   24
#define PIN_SW_LIMIT_ELEVATION_TOP      25
#define PIN_BUTTON_START                26
#define PIN_LED_AZIMUTH_LEFT            38
#define PIN_LED_AZIMUTH_CENTER          39
#define PIN_LED_AZIMUTH_RIGHT           40
#define PIN_LED_ELEVATION_BOTTOM        42
#define PIN_LED_ELEVATION_CENTER        43
#define PIN_LED_ELEVATION_TOP           44
#define PIN_LED_OK                      46
#define PIN_LED_ERROR                   47
#define PIN_LCD_RS                      48
#define PIN_LCD_EN                      49
#define PIN_LCD_D4                      50
#define PIN_LCD_D5                      51
#define PIN_LCD_D6                      52
#define PIN_LCD_D7                      53



// Stepper motors: Define steps per revolution and speed in RPM.
#ifdef SIMULATION_MODE
#define STEPS_AZIMUTH   64
#define STEPS_ELEVATION 64
#define SPEED_AZIMUTH   10    // RPM
#define SPEED_ELEVATION 10    // RPM
#else
#define STEPS_AZIMUTH   2048
#define STEPS_ELEVATION 2048
#define SPEED_AZIMUTH   10    // RPM
#define SPEED_ELEVATION 10    // RPM
#endif // SIMULATION_MODE

// Create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to.
Stepper stepperAzimuth(STEPS_AZIMUTH, 30, 32, 31, 33);
Stepper stepperElevation(STEPS_ELEVATION, 34, 36, 35, 37);
int millisPerStepAzimuth = (float) 60 / (STEPS_AZIMUTH * SPEED_AZIMUTH) * 1000;
int millisPerStepElevation = (float) 60 / (STEPS_ELEVATION * SPEED_ELEVATION) * 1000;
int positionAzimuth;
int positionElevation;

// Create an instance of the LCD.
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

// Azimuth and elevation software limits.
float azimuthLimitMin     = 10.0;
float azimuthLimitMax     = 350.0;
float elevationLimitMin   = 10.0;
float elevationLimitMax   = 80.0;

// Azimuth and elevation actual value, target value and tolerance.
float azimuthActual       = 0.0;
float azimuthTarget       = 0.0;
float azimuthTolerance    = 20.0;
float elevationActual     = 0.0;
float elevationTarget     = 0.0;
float elevationTolerance  = 10.0;



void setup() {
  // Initialize the serial interface.
  Serial.begin(9600);
  // Print a message on the serial interface.
  Serial.print("\r\n*** Welcome to the MAGIC Game: Catch the Gamma! ***");
  Serial.print("\r\n");
  Serial.print("\r\nFirmware info:");
  Serial.print("\r\n- Name: ");
  Serial.print(FW_NAME);
  Serial.print("\r\n- Ver.: ");
  Serial.print(FW_VERSION);
  Serial.print("\r\n- Date: ");
  Serial.print(FW_RELEASEDATE);
  Serial.print("\r\n");

  // Set the speed of the stepper motors.
  stepperAzimuth.setSpeed(SPEED_AZIMUTH);
  stepperElevation.setSpeed(SPEED_ELEVATION);

  // Set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message on the LCD.
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
  Serial.print("\r\nPlease press the start button to continue.");
  waitStart();
  delay(1000);
}



void loop() {
  initHardware();
  while (true) {
    initGame();
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

  #ifdef SIMULATION_MODE
  // Do not try to find the zero positions of the stepper motors in simulation mode.
  ret = 0;
  #else
  // Find the zero positions of the stepper motors.
  Serial.print("\r\nFinding the zero positions of the stepper motors.");
  ret = stepperFindZeroPosition();
  #endif
  // Error while finding the zero positions.
  // => Halt the execution and display an error message.
  if (ret) {
    digitalWrite(PIN_LED_OK, LOW);
    digitalWrite(PIN_LED_ERROR, HIGH);
    Serial.print("\r\nERROR: Zero position of stepper motors not found! Program stopped!");
    Serial.print("\r\nPress the reset button to reboot.");
    Serial.print("\r\n");
    lcd.clear();
    while (true) {
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
  int steps;
  // Azimuth: Move to left until limit switch gets activated.
  steps = 0;
  while (digitalRead(PIN_SW_LIMIT_AZIMUTH_LEFT)) {
    stepperAzimuth.step(1);
    steps++;
    // Error: One complete rotation without limit switch getting activated!
    if (steps > STEPS_AZIMUTH) return 1;
  }
  // Elevation: Move down until limit switch gets activated.
  steps = 0;
  while (digitalRead(PIN_SW_LIMIT_ELEVATION_BOTTOM)) {
    stepperElevation.step(1);
    steps++;
    // Error: Half a rotation without limit switch getting activated!
    if (steps > STEPS_ELEVATION / 2) return 2;
  }

  positionAzimuth = 0;
  positionElevation = 0;

  return 0;
}



// Initialize the game.
int initGame() {
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

  // Display message.
  Serial.print("\r\n\r\nMAGIC Game: Push the start button to start a new game.\r\n");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("-= MAGIC Game =-");
  lcd.setCursor(0, 1);
  lcd.print("* Push button! *");

  // Wait until the start button is pushed.
  waitStart();

  // Generate a new gamma position.
  azimuthTarget = random(azimuthLimitMin, azimuthLimitMax);
  elevationTarget = random(elevationLimitMin, elevationLimitMax);

  // Display the gamma position.
  Serial.print("\r\nNew gamma position: Azimuth: " + String(int(azimuthTarget)) + ", elevation: " + String(int(elevationTarget)));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Gamma position:");
  lcd.setCursor(0, 1);
  lcd.print("Az: " + String(int(azimuthTarget)) + " , el: " + String(int(elevationTarget)));
  return 0;
}



// Play the game.
int playGame() {
  int ret;
  int stepsAzimuth;
  int stepsElevation;
  int stepsMax;
  long timeStart;
  long timeElapsed;

  #ifdef DEBUG_MODE_0
  lcd.clear();
  #endif

  timeStart = millis();

  while (true) {
    // Show the progress of the physics event (gamma ray -> Cherenkov light).
    timeElapsed = millis() - timeStart;
    #ifdef SIMULATION_MODE
    if (timeElapsed > 250) digitalWrite(PIN_LED_PROGRESS_1, HIGH);
    if (timeElapsed > 3000) digitalWrite(PIN_LED_PROGRESS_2, HIGH);
    if (timeElapsed > 6000) digitalWrite(PIN_LED_PROGRESS_3, HIGH);
    if (timeElapsed > 9000) digitalWrite(PIN_LED_PROGRESS_4, HIGH);
    if (timeElapsed > 12000) digitalWrite(PIN_LED_PROGRESS_5, HIGH);
    if (timeElapsed > 15000) {
    #else
    if (timeElapsed > 250) digitalWrite(PIN_LED_PROGRESS_1, HIGH);
    if (timeElapsed > 1000) digitalWrite(PIN_LED_PROGRESS_2, HIGH);
    if (timeElapsed > 2000) digitalWrite(PIN_LED_PROGRESS_3, HIGH);
    if (timeElapsed > 3000) digitalWrite(PIN_LED_PROGRESS_4, HIGH);
    if (timeElapsed > 4000) digitalWrite(PIN_LED_PROGRESS_5, HIGH);
    if (timeElapsed > 5000) {
    #endif
      ret = evalGameResult();
      return ret;
    }

    // Get the values from the analog joystick.
    stepsAzimuth = analog2steps(analogRead(PIN_JOYSTICK_AZ));
    stepsElevation = analog2steps(analogRead(PIN_JOYSTICK_EL));

    // DEBUG: Show steps for azimuth and elevation.
    #ifdef DEBUG_MODE_0
    #ifdef DEBUG_SERIAL
    Serial.print("\r\nDEBUG: Steps azimuth: " + String(stepsAzimuth) + ", steps elevation: " + String(stepsElevation));
    #endif
    lcd.setCursor(0, 0);
    lcd.print("DBG stp. az: " + String(stepsAzimuth) + "  ");
    lcd.setCursor(0, 1);
    lcd.print("DBG stp. el: " + String(stepsElevation) + "  ");
    #endif

    // Move the stepper motors.
//    stepperAzimuth.step(stepsAzimuth);
//    stepperElevation.step(stepsElevation);
    // Move both motors in parallel step by step.
    stepsMax = max(abs(analog2steps(0)), abs(analog2steps(1024)));  // Get the maximum number of possible steps.
    for (int i = 0; i < stepsMax; i++) {
      // Convert stepper motor positions to azimuth and elevation.
      azimuthActual = position2azimuth(positionAzimuth);
      elevationActual = position2elevation(positionElevation);

      // Azimuth: Rotate right.
      if ((stepsAzimuth < 0) && (azimuthActual < azimuthLimitMax) && (digitalRead(PIN_SW_LIMIT_AZIMUTH_RIGHT))) {
        stepperAzimuth.step(-1);
        stepsAzimuth++;
        positionAzimuth++;
      // Azimuth: Rotate left.
      } else if ((stepsAzimuth > 0) && (azimuthActual > azimuthLimitMin) && (digitalRead(PIN_SW_LIMIT_AZIMUTH_LEFT))) {
        stepperAzimuth.step(1);
        stepsAzimuth--;
        positionAzimuth--;
      // Azimuth: No rotation, but insert delay to compensate the missing time.
      } else {
        delay(millisPerStepAzimuth);
      }

      // Elevation: Move up.
      if ((stepsElevation < 0) && (elevationActual < elevationLimitMax) && (digitalRead(PIN_SW_LIMIT_ELEVATION_TOP))) {
        stepperElevation.step(-1);
        stepsElevation++;
        positionElevation++;
      // Elevation: Move down.
      } else if ((stepsElevation > 0) && (elevationActual > elevationLimitMin) && (digitalRead(PIN_SW_LIMIT_ELEVATION_BOTTOM))) {
        stepperElevation.step(1);
        stepsElevation--;
        positionElevation--;
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
    #ifdef DEBUG_MODE_1
    #ifdef DEBUG_SERIAL
    Serial.print("\r\nDEBUG: Actual azimuth: " + String(int(azimuthActual)) + ", actual elevation: " + String(int(elevationActual)));
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
int analog2steps(int analog) {
  int steps = 0;
  if (analog < 200)       steps = +4;
  else if (analog < 300)  steps = +2;
  else if (analog < 400)  steps = +1;
  else if (analog < 624)  steps = +0;
  else if (analog < 724)  steps = -1;
  else if (analog < 824)  steps = -2;
  else                    steps = -4;
  return steps;
}



// Convert stepper motor position to azimuth.
float position2azimuth(int steps) {
  return ((float) steps / STEPS_AZIMUTH) * 360;
}



// Convert stepper motor position to elevation.
float position2elevation(int steps) {
  return ((float) steps / STEPS_ELEVATION) * 360;
}



// Evaluate the result of the game.
int evalGameResult() {
  int ret;

  // Display evaluation.
  lcd.clear();
  if ((azimuthActual > azimuthTarget - azimuthTolerance) && (azimuthActual < azimuthTarget + azimuthTolerance) &&
      (elevationActual > elevationTarget - elevationTolerance) && (elevationActual < elevationTarget + elevationTolerance)
  ) {
    Serial.print("\r\nCongratulations! You caught the gamma!");
    lcd.setCursor(0, 0);
    lcd.print("Congratulations!");
    lcd.setCursor(0, 1);
    lcd.print("Gamma caught!");
    ret = 0;
  } else {
    Serial.print("\r\nSorry, You missed the gamma!");
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
