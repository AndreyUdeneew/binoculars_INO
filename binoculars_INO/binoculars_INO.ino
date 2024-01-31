/****************************************************************************************************************************
  Argument_Simple.ino
  For MBED RP2040-based boards such as Nano_RP2040_Connect, RASPBERRY_PI_PICO, ADAFRUIT_FEATHER_RP2040 and GENERIC_RP2040.
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/MBED_RPI_PICO_TimerInterrupt
  Licensed under MIT license

  The RPI_PICO system timer peripheral provides a global microsecond timebase for the system, and generates
  interrupts based on this timebase. It supports the following features:
    • A single 64-bit counter, incrementing once per microsecond
    • This counter can be read from a pair of latching registers, for race-free reads over a 32-bit bus.
    • Four alarms: match on the lower 32 bits of counter, IRQ on match: TIMER_IRQ_0-TIMER_IRQ_3

  Now even you use all these new 16 ISR-based timers,with their maximum interval practically unlimited (limited only by
  unsigned long miliseconds), you just consume only one RPI_PICO timer and avoid conflicting with other cores' tasks.
  The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
  Therefore, their executions are not blocked by bad-behaving functions / tasks.
  This important feature is absolutely necessary for mission-critical tasks.
 *****************************************************************************************************************************/

#if (defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || defined(ARDUINO_GENERIC_RP2040)) && defined(ARDUINO_ARCH_MBED)
#define USING_MBED_RPI_PICO_TIMER_INTERRUPT true
#else
#error This code is intended to run on the MBED RASPBERRY_PI_PICO platform! Please check your Tools->Board setting.
#endif

// These define's must be placed at the beginning before #include "TimerInterrupt_Generic.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
#define _TIMERINTERRUPT_LOGLEVEL_ 4

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "MBED_RPi_Pico_TimerInterrupt.h"
#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#define IRQ_PIN 3
#define XSHUT_PIN 28
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

String cmd, CMDcur;
uint8_t programNumber;

volatile int counter = 0;

uint8_t strobeInput = 2;
uint8_t UV_LED = 8;
uint8_t WHITE_LED = 6;
uint8_t RED_LED = 7;
uint8_t IR_LED = 28;

uint8_t VAR_X_pin = 26;
uint8_t VAR_Y_pin = 27;

uint8_t nMotorsSleep = 9;

uint8_t nFLT1 = 10;
uint8_t nEnl = 11;
uint8_t stepPin_1 = 12;
uint8_t dirPin_1 = 13;
uint8_t k1 = 14;
uint8_t k2 = 15;

uint8_t nFLT2 = 21;
uint8_t nEn2 = 20;
uint8_t stepPin_2 = 19;
uint8_t dirPin_2 = 18;
uint8_t solenoid_DIR = 16;
uint8_t solenoid_ON = 17;

volatile uint8_t PWM_White = 255;
volatile uint8_t PWM_UV = 255;
volatile uint8_t PWM_Red = 255;
volatile uint8_t PWM_IR = 255;

uint8_t mode;
uint8_t actualFilter = 0;

uint16_t VAR_X = 0;
uint16_t VAR_Y = 0;

int zoomPosition;
int focusPosition;
int zoomTargetPosition;
int focusTargetPsition;
int maxFocusSteps = 2100;
int maxZoomSteps = 2900;

int16_t distance;

volatile uint8_t M[4][2];
volatile uint8_t M0[4][2]{
  { PWM_White, 0 },
  { PWM_White, 0 },
  { PWM_White, 0 },
  { PWM_White, 0 }
};
volatile uint8_t M1[4][2];
volatile uint8_t M2[4][2];
volatile uint8_t M3[4][2];
volatile uint8_t M4[4][2];
volatile uint8_t M5[4][2];
volatile uint8_t M6[4][2];
volatile uint8_t M7[4][2];

int autofocusTable[256][256];  // autofocusTable[distance, mm][zoomPosition, steps];


// Init RPI_PICO_Timer
MBED_RPI_PICO_Timer ITimer1(1);

#define TIMER1_INTERVAL_MS 50

// Never use Serial.print inside this mbed ISR. Will hang the system
void TimerHandler1(uint alarm_num) {
  static bool toggle1 = false;

  ///////////////////////////////////////////////////////////
  // Always call this for MBED RP2040 before processing ISR
  TIMER_ISR_START(alarm_num);
  ///////////////////////////////////////////////////////////

  // digitalWrite(outputPin1, toggle1);
  toggle1 = !toggle1;
  if (actualFilter == 1) {
    actualFilter = 0;
  } else {
    actualFilter = 1;
  }
  // distance = vl53.distance();

  ////////////////////////////////////////////////////////////
  // Always call this for MBED RP2040 after processing ISR
  TIMER_ISR_END(alarm_num);
  ////////////////////////////////////////////////////////////
}

void modesCacheRefresh() {
  // Serial.println("cache refreshing");
  M1[0][0] = PWM_UV;
  M1[0][1] = 0;
  M1[1][0] = 0;
  M1[1][1] = 0;
  M1[2][0] = PWM_White;
  M1[2][1] = PWM_White;
  M1[3][0] = 0;
  M1[3][1] = 0;

  M2[0][0] = 0;
  M2[0][1] = 0;
  M2[1][0] = PWM_Red;
  M2[1][1] = 0;
  M2[2][0] = PWM_White;
  M2[2][1] = PWM_White;
  M2[3][0] = 0;
  M2[3][1] = 0;

  M3[0][0] = PWM_UV;  //Both UV and Red LEDs
  M3[0][1] = 0;
  M3[1][0] = PWM_Red;
  M3[1][1] = 0;
  M3[2][0] = PWM_White;
  M3[2][1] = PWM_White;
  M3[3][0] = 0;
  M3[3][1] = 0;

  M4[0][0] = PWM_UV;  //oxygenation IR LEDs must be mounted instead of UV LEDs.
  M4[0][1] = 0;
  M4[1][0] = 0;
  M4[1][1] = PWM_Red;
  M4[2][0] = 0;
  M4[2][1] = 0;
  M4[3][0] = 0;
  M4[3][1] = 0;

  M5[0][0] = 0;  // ICG mode IR LEDs must be mounted instead of White LEDs.
  M5[0][1] = 0;
  M5[1][0] = 0;
  M5[1][1] = 0;
  M5[2][0] = PWM_White;
  M5[2][1] = 0;
  M4[3][0] = 0;
  M4[3][1] = 0;

  M6[0][0] = PWM_UV;  //Sequental stroboscope of red and UV LEDs.
  M6[0][1] = 0;
  M6[1][0] = 0;
  M6[1][1] = PWM_Red;
  M6[2][0] = PWM_White;
  M6[2][1] = PWM_White;
  M6[3][0] = 0;
  M6[3][1] = 0;

  M7[0][0] = 0;
  M7[0][1] = 0;
  M7[1][0] = 0;
  M7[1][1] = 0;
  M7[2][0] = PWM_White;
  M7[2][1] = PWM_White;
  M7[3][0] = PWM_IR;
  M7[3][1] = 0;
}

void setup() {
  M1[0][0] = PWM_UV;
  M1[0][1] = 0;
  M1[1][0] = 0;
  M1[1][1] = 0;
  M1[2][0] = PWM_White;
  M1[2][1] = PWM_White;
  M1[3][0] = 0;
  M1[3][1] = 0;

  M2[0][0] = 0;
  M2[0][1] = 0;
  M2[1][0] = PWM_Red;
  M2[1][1] = 0;
  M2[2][0] = PWM_White;
  M2[2][1] = PWM_White;
  M2[3][0] = 0;
  M2[3][1] = 0;

  M3[0][0] = PWM_UV;
  M3[0][1] = 0;
  M3[1][0] = PWM_Red;
  M3[1][1] = 0;
  M3[2][0] = PWM_White;
  M3[2][1] = PWM_White;
  M3[3][0] = 0;
  M3[3][1] = 0;

  M4[0][0] = PWM_UV;  //oxygenation IR LEDs must be mounted instead of UV LEDs.
  M4[0][1] = 0;
  M4[1][0] = 0;
  M4[1][1] = PWM_Red;
  M4[2][0] = 0;
  M4[2][1] = 0;
  M4[3][0] = 0;
  M4[3][1] = 0;

  M5[0][0] = 0;
  M5[0][1] = 0;
  M5[1][0] = 0;
  M5[1][1] = 0;
  M5[2][0] = PWM_White;
  M5[2][1] = 0;
  M5[3][0] = 0;
  M5[3][1] = 0;

  M6[0][0] = PWM_UV;  //Sequental stroboscope of red and UV LEDs.
  M6[0][1] = 0;
  M6[1][0] = 0;
  M6[1][1] = PWM_Red;
  M6[2][0] = PWM_White;
  M6[2][1] = PWM_White;
  M6[3][0] = 0;
  M6[3][1] = 0;

  M7[0][0] = 0;  // ICG mode IR LEDs must be mounted instead of White LEDs.
  M7[0][1] = 0;
  M7[1][0] = 0;
  M7[1][1] = 0;
  M7[2][0] = PWM_White;
  M7[2][1] = PWM_White;
  M7[3][0] = PWM_IR;
  M7[3][1] = 0;

  pinMode(stepPin_1, OUTPUT);
  pinMode(dirPin_1, OUTPUT);
  pinMode(stepPin_2, OUTPUT);
  pinMode(dirPin_2, OUTPUT);
  pinMode(nMotorsSleep, OUTPUT);
  pinMode(nEnl, OUTPUT);
  pinMode(nEn2, OUTPUT);
  pinMode(nFLT1, INPUT);
  pinMode(nFLT2, INPUT);
  pinMode(k1, INPUT);
  pinMode(k2, INPUT);
  pinMode(solenoid_DIR, OUTPUT);
  pinMode(solenoid_ON, OUTPUT);

  //  pinMode(strpbeInput, INPUT_PULLUP); /// Our camera strobe in HIGH - Acquiring, LOW - not acquiring
  pinMode(UV_LED, OUTPUT);     // UV LED
  pinMode(RED_LED, OUTPUT);    // UV LED
  pinMode(WHITE_LED, OUTPUT);  // White LED
  pinMode(IR_LED, OUTPUT);     // White LED
  pinMode(3, OUTPUT);          // For migalka test

  analogWrite(WHITE_LED, PWM_White);
  delay(10);
  analogWrite(UV_LED, PWM_White);   // 4 correct work of interrpt
  analogWrite(RED_LED, PWM_White);  // 4 correct work of interrpt
  analogWrite(IR_LED, PWM_White);   // 4 correct work of interrpt
  //  analogWrite(UV_LED, PWM_White); // 4 correct work of interrpt
  //digitalWrite(UV_LED, HIGH);// 4 correct work of interrpt
  //digitalWrite(RED_LED, HIGH);// 4 correct work of interrpt
  Serial.begin(115200);
  Serial.setTimeout(100);
  //  pinMode(strobeInput,INPUT);
  //  attachInterrupt(strobeInput, Strobe_Input_Handler, RISING); // 4 ARDUINO
  attachInterrupt(digitalPinToInterrupt(strobeInput), Strobe_Input_Handler, RISING);  // 4 Rpi Pico
  //  pinMode(strobeInput, INPUT_PULLUP); // 4 Rpi Pico pull_up must be after the attachinterrupt. It's a bug.
  pinMode(strobeInput, INPUT);  // 4 Rpi Pico pull_up must be after the attachinterrupt. It's a bug.
  digitalWrite(solenoid_DIR, LOW);
  digitalWrite(solenoid_ON, LOW);
  motorsCalibration();
  while (!Serial) delay(10);
  //
  Serial.println(F("Adafruit VL53L1X sensor demo"));
  Wire.begin(400000);
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("VL53L1X sensorOK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("Ranging started"));

  //   Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(15);
  //    Serial.print(F("Timing budget (ms): "));
  //    Serial.println(vl53.getTimingBudget());


  //  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  //  vl.VL53L1X_SetInterruptPolarity(0);



  if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, TimerHandler1)) {
    Serial.print(F("Starting ITimer1 OK, millis() = "));
    Serial.println(millis());

#if (TIMER_INTERRUPT_DEBUG > 1)
    Serial.print(F("OutputPin1 = "));
    Serial.print(outputPin1);
#endif
  } else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));

  zoomNsteps(1, maxZoomSteps, 1);    // correct N of steps
  focusNsteps(1, maxFocusSteps, 1);  // correct N of steps dir 1 - to the closest zoom
  zoomPosition = 0;
  focusPosition = 0;
}

void motorsCalibration() {
  digitalWrite(nEnl, LOW);
  digitalWrite(dirPin_1, HIGH);

  digitalWrite(nEn2, LOW);
  digitalWrite(dirPin_2, LOW);

  digitalWrite(nMotorsSleep, LOW);
  //  Motor1 - focus(?)
  //  Motor2 - fzoom(?)
  //    for (int x = 0; x < 65536; x++) {
  //      digitalWrite(stepPin_1, HIGH);
  //      digitalWrite(stepPin_2, HIGH);
  //      delay(5);  // ms (Note : 1000ms = 1sec)
  //      digitalWrite(stepPin_1, LOW);
  //      digitalWrite(stepPin_2, LOW);
  //      delay(5); // ms (Note : 1000ms = 1sec)
  //      Serial.println(x);
  //}
}

void Strobe_Input_Handler() {
  if (counter == 2) {
    counter = 0;
  }
  if (counter == 1) {
    //    analogWrite(UV_LED, PWM_UV);
    //    analogWrite(RED_LED, 0);
    analogWrite(UV_LED, M[0][0]);
    analogWrite(RED_LED, M[1][0]);
    analogWrite(WHITE_LED, M[2][0]);
    analogWrite(IR_LED, M[3][0]);
  } else {
    //    analogWrite(UV_LED, 0);
    //    analogWrite(RED_LED, PWM_Red);
    analogWrite(UV_LED, M[0][1]);
    analogWrite(RED_LED, M[1][1]);
    analogWrite(WHITE_LED, M[2][1]);
    analogWrite(IR_LED, M[3][1]);
  }

  counter += 1;  // + синхр.
}

void waiting_4_command() {
  int PWM_VAL, PWM_VALH, PWM_VALL, PWM_VALlowest;
  cmd = "";
  if (Serial.available()) {
    //    cmd = Serial.readStringUntil('\n');
    cmd = Serial.readString();
    cmd.trim();
  }


  if (cmd.substring(0, 2) == "UV") {
    PWM_VALH = cmd[2] - '0';
    PWM_VALL = cmd[3] - '0';
    PWM_VALlowest = cmd[4] - '0';
    PWM_VAL = (PWM_VALH * 100) + (PWM_VALL * 10) + (PWM_VALlowest * 1);
    PWM_UV = PWM_VAL;
    modesCacheRefresh();
    //        analogWrite(UV_LED, PWM_UV);
    // Serial.println("UV has been changed, modes cache was refreshed");
    // Serial.println(PWM_VAL);
  }

  if (cmd.substring(0, 2) == "WH") {
    PWM_VALH = cmd[2] - '0';
    PWM_VALL = cmd[3] - '0';
    PWM_VALlowest = cmd[4] - '0';
    PWM_VAL = (PWM_VALH * 100) + (PWM_VALL * 10) + (PWM_VALlowest * 1);
    PWM_White = PWM_VAL;
    modesCacheRefresh();
    analogWrite(WHITE_LED, PWM_White);
    // Serial.println("WH has been changed, modes cache was refreshed");
    // Serial.println(PWM_VAL);
  }

  if (cmd.substring(0, 2) == "RE") {
    PWM_VALH = cmd[2] - '0';
    PWM_VALL = cmd[3] - '0';
    PWM_VALlowest = cmd[4] - '0';
    PWM_VAL = (PWM_VALH * 100) + (PWM_VALL * 10) + (PWM_VALlowest * 1);
    PWM_Red = PWM_VAL;
    modesCacheRefresh();
    //    analogWrite(RED_LED, PWM_Red);
    // Serial.println("RE has been changed, modes cache was refreshed");
    // Serial.println(PWM_VAL);
  }

  if (cmd.substring(0, 2) == "FC") {
    actualFilter = cmd[2] - '0';
    filterChange(actualFilter);
  }

  if (cmd.substring(0, 5) == "DIST?") {
    distanceMeas();
  }

  if (cmd.substring(0, 3) == "OFF") {
    zoomNsteps(0, maxZoomSteps, 1);    // correct N of steps
    focusNsteps(0, maxFocusSteps, 1);  // correct N of steps dir 1 - to the closest zoom
  }

  if (cmd.substring(0, 4) == "ZOOM") {
    uint8_t dir;
    if (cmd[4] == '+') {
      dir = 0;
    } else {
      dir = 1;
    }
    zoomNsteps(dir, 100, 1);  // correct N of steps
  }

  if (cmd.substring(0, 5) == "FOCUS") {
    uint8_t dir;
    if (cmd[5] == '-') {
      dir = 0;
    } else {
      dir = 1;
    }
    focusNsteps(dir, 100, 1);  // correct N of steps
  }

  if (cmd.substring(0, 1) == "M") {
    mode = cmd[1] - '0';
    // Serial.println("mode has been changed");
    // Serial.println(mode);
    if (mode == 1) {

      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M1[i][j];
    }
    if (mode == 2) {

      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M2[i][j];
    }
    if (mode == 3) {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M3[i][j];
    }
    if (mode == 4) {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M4[i][j];
    }
    if (mode == 5) {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M5[i][j];
    }
    if (mode == 6) {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M6[i][j];
    }
    if (mode == 7) {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M7[i][j];
    }
    if (mode == 0) {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M0[i][j];
    }
  }
}

int distanceMeas(void) {

  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return 0;
    }
    // Serial.print(F("Distance: "));
    Serial.print(distance);
    // Serial.println(" mm");

    // data is read out, time for another reading!
    vl53.clearInterrupt();
  }
  return distance;
}

void filterChange(uint8_t actualFilter) {
  // Serial.println("filter changing");
  // Serial.println(actualFilter);
  digitalWrite(solenoid_ON, HIGH);
  delay(5);
  if (actualFilter == 0) {
    digitalWrite(solenoid_DIR, HIGH);
    delay(5);
  } else {
    digitalWrite(solenoid_DIR, LOW);
    delay(5);
  }
  digitalWrite(solenoid_ON, LOW);
}

void zoom(uint8_t dir, uint8_t lag) {
  // Serial.println("filter switching");
  // Serial.println(actualFilter);
  digitalWrite(nMotorsSleep, HIGH);
  uint32_t zoomCount = 0;
  if (dir == 1) {  //dir 1 - to the closest zoom
    digitalWrite(dirPin_2, HIGH);
    while (analogRead(VAR_Y_pin) >= 767) {
      digitalWrite(stepPin_2, HIGH);
      delay(lag);
      digitalWrite(stepPin_2, LOW);
      delay(lag);
      zoomCount += 1;
      zoomPosition += 1;
      if (zoomPosition >= maxZoomSteps) {
        zoomPosition = maxZoomSteps;
      }
      // Serial.println(zoomCount);
    }
  }
  if (dir == 0) {
    digitalWrite(dirPin_2, LOW);
    while (analogRead(VAR_Y_pin) <= 256) {
      digitalWrite(stepPin_2, HIGH);
      delay(lag);
      digitalWrite(stepPin_2, LOW);
      delay(lag);
      zoomCount += 1;
      zoomPosition -= 1;
      if (zoomPosition <= 0) {
        zoomPosition = 0;
      }
      // Serial.println(zoomCount);
    }
  }
  digitalWrite(nMotorsSleep, LOW);
}

void zoomNsteps(uint8_t dir, int nSteps, uint8_t lag) {
  // Serial.println("filter switching");
  // Serial.println(actualFilter);
  digitalWrite(nMotorsSleep, HIGH);
  uint32_t zoomCount = 0;
  if (dir == 1) {  //dir 1 - to the closest zoom
    digitalWrite(dirPin_2, HIGH);
    for (int i = 0; i < nSteps; i++) {
      digitalWrite(stepPin_2, HIGH);
      delay(lag);
      digitalWrite(stepPin_2, LOW);
      delay(lag);
      zoomCount += 1;
      zoomPosition += 1;
      if (zoomPosition >= maxZoomSteps) {
        zoomPosition = maxZoomSteps;
      }
      // Serial.println(zoomCount);
    }
  }
  if (dir == 0) {
    digitalWrite(dirPin_2, LOW);
    for (int i = 0; i < nSteps; i++) {
      digitalWrite(stepPin_2, HIGH);
      delay(lag);
      digitalWrite(stepPin_2, LOW);
      delay(lag);
      zoomCount += 1;
      zoomPosition -= 1;
      if (zoomPosition <= 0) {
        zoomPosition = 0;
      }
      // Serial.println(zoomCount);
    }
  }
  digitalWrite(nMotorsSleep, LOW);
}

void focus(uint8_t dir, uint8_t lag) {
  digitalWrite(nMotorsSleep, HIGH);
  uint32_t focusCount = 0;
  if (dir == 1) {
    digitalWrite(dirPin_1, HIGH);
    while (analogRead(VAR_X_pin) >= 767) {
      digitalWrite(stepPin_1, HIGH);
      delay(lag);
      digitalWrite(stepPin_1, LOW);
      delay(lag);
      focusCount += 1;
      focusPosition += 1;
      if (focusPosition >= maxFocusSteps) {
        focusPosition = maxFocusSteps;
      }
      // Serial.println(focusCount);
    }
  }
  if (dir == 0) {
    digitalWrite(dirPin_1, LOW);
    while (analogRead(VAR_X_pin) <= 256) {
      digitalWrite(stepPin_1, HIGH);
      delay(lag);
      digitalWrite(stepPin_1, LOW);
      delay(lag);
      focusCount -= 1;
      focusPosition -= 1;
      if (focusPosition <= 0) {
        focusPosition = 0;
      }
      // Serial.println(focusCount);
    }
  }
  digitalWrite(nMotorsSleep, LOW);
}

void focusNsteps(uint8_t dir, int nSteps, uint8_t lag) {
  digitalWrite(nMotorsSleep, HIGH);
  uint32_t focusCount = 0;
  if (dir == 1) {
    digitalWrite(dirPin_1, HIGH);
    for (int i = 0; i < nSteps; i++) {
      digitalWrite(stepPin_1, HIGH);
      delay(lag);
      digitalWrite(stepPin_1, LOW);
      delay(lag);
      focusCount += 1;
      focusPosition += 1;
      if (focusPosition >= maxFocusSteps) {
        focusPosition = maxFocusSteps;
      }
      // Serial.println(focusCount);
    }
  }
  if (dir == 0) {
    digitalWrite(dirPin_2, LOW);
    for (int i = 0; i < nSteps; i++) {
      digitalWrite(stepPin_1, HIGH);
      delay(lag);
      digitalWrite(stepPin_1, LOW);
      delay(lag);
      focusCount += 1;
      focusPosition -= 1;
      if (focusPosition <= 0) {
        focusPosition = 0;
      }
      // Serial.println(focusCount);
    }
  }
  digitalWrite(nMotorsSleep, LOW);
}

void focusCorrection() {
  int dir;
  int distance = distanceMeas();
  int distanceRange = round(distance/10);
  int zoomPositionRange = round(zoomPosition/10);
  int correctFocus = autofocusTable[distanceRange][zoomPosition];
  int deltaFocus = abs(correctFocus - focusPosition);
  int steps = deltaFocus + focusPosition;
  focusNsteps(dir, steps, 1);
}

void loop() {
  int lastTimer1;
  static bool timer1Stopped = false;

  if (millis() - lastTimer1 > TIMER1_INTERVAL_MS) {
    lastTimer1 = millis();

    if (timer1Stopped) {
      // Serial.print(F("Start ITimer1, millis() = "));
      // Serial.println(millis());
      ITimer1.restartTimer();
      distanceMeas();
      // focuscorrection();
      Serial.println(zoomPosition);
      Serial.println(focusPosition);
      // filterChange(actualFilter);
    } else {
      // Serial.print(F("Stop ITimer1, millis() = "));
      // Serial.println(millis());
      ITimer1.stopTimer();
    }

    timer1Stopped = !timer1Stopped;
    // timer1Stopped = timer1Stopped;

    // focusNsteps(0, 500, 1);
    // delay(1000);
    // focusNsteps(1, 500, 1);
    // delay(1000);
    // zoomNsteps(0, 500, 1);
    // delay(1000);
    // zoomNsteps(1, 500, 1);
    // delay(1000);


    // Serial.print("Distance = ");
    // Serial.println(distance);
    // delay(1000);

    VAR_X = analogRead(VAR_X_pin);
    VAR_Y = analogRead(VAR_Y_pin);

    if ((VAR_Y >= 767)) {
      zoom(1, 2);
    }
    if (VAR_Y <= 256) {
      zoom(0, 2);
    }

    if ((VAR_X >= 767)) {
      focus(1, 2);
    }
    if (VAR_X <= 256) {
      focus(0, 2);
    }

    //  Serial.print("X = ");
    //  Serial.print(VAR_X);
    //  Serial.print("\t Y = ");
    //  Serial.println(VAR_Y);

    //  delay(20);
    // Serial.println(counter);
    // digitalWrite(3, HIGH);
    //  filterChange(0);
    // delay(500);
    //  digitalWrite(3, LOW);
    //  filterChange(1);
    // delay(500);
    if (Serial.available()) {
      waiting_4_command();
    }
  }
}