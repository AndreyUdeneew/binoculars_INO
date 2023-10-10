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

volatile uint8_t PWM_White = 10;
volatile uint8_t PWM_UV = 1;
volatile uint8_t PWM_Red = 1;
volatile uint8_t PWM_IR = 1;

uint8_t mode;
uint8_t actualFilter = 0;

uint16_t VAR_X = 0;
uint16_t VAR_Y = 0;

volatile uint8_t M[4][2];
volatile uint8_t M0[4][2]
{
  {PWM_White, 0},
  {PWM_White, 0},
  {PWM_White, 0},
  {PWM_White, 0}
};
volatile uint8_t M1[4][2];
volatile uint8_t M2[4][2];
volatile uint8_t M3[4][2];
volatile uint8_t M4[4][2];
volatile uint8_t M5[4][2];
volatile uint8_t M6[4][2];
volatile uint8_t M7[4][2];

void modesCacheRefresh()
{
  Serial.println("cache refreshing");
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

  M3[0][0] = PWM_UV;      //Both UV and Red LEDs
  M3[0][1] = 0;
  M3[1][0] = PWM_Red;
  M3[1][1] = 0;
  M3[2][0] = PWM_White;
  M3[2][1] = PWM_White;
  M3[3][0] = 0;
  M3[3][1] = 0;

  M4[0][0] = PWM_UV;          //oxygenation IR LEDs must be mounted instead of UV LEDs.
  M4[0][1] = 0;
  M4[1][0] = 0;
  M4[1][1] = PWM_Red;
  M4[2][0] = 0;
  M4[2][1] = 0;
  M4[3][0] = 0;
  M4[3][1] = 0;

  M5[0][0] = 0;             // ICG mode IR LEDs must be mounted instead of White LEDs.
  M5[0][1] = 0;
  M5[1][0] = 0;
  M5[1][1] = 0;
  M5[2][0] = PWM_White;
  M5[2][1] = 0;
  M4[3][0] = 0;
  M4[3][1] = 0;

  M6[0][0] = PWM_UV;          //Sequental stroboscope of red and UV LEDs.
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

void setup()
{
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

  M4[0][0] = PWM_UV;          //oxygenation IR LEDs must be mounted instead of UV LEDs.
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

  M6[0][0] = PWM_UV;          //Sequental stroboscope of red and UV LEDs.
  M6[0][1] = 0;
  M6[1][0] = 0;
  M6[1][1] = PWM_Red;
  M6[2][0] = PWM_White;
  M6[2][1] = PWM_White;
  M6[3][0] = 0;
  M6[3][1] = 0;

  M7[0][0] = 0;             // ICG mode IR LEDs must be mounted instead of White LEDs.
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
  pinMode(UV_LED, OUTPUT);// UV LED
  pinMode(RED_LED, OUTPUT);// UV LED
  pinMode(WHITE_LED, OUTPUT);// White LED
  pinMode(IR_LED, OUTPUT);// White LED
  pinMode(3, OUTPUT);// For migalka test

  analogWrite(WHITE_LED, PWM_White);
  delay(10);
  analogWrite(UV_LED, PWM_White); // 4 correct work of interrpt
  analogWrite(RED_LED, PWM_White); // 4 correct work of interrpt
  analogWrite(IR_LED, PWM_White); // 4 correct work of interrpt
  //  analogWrite(UV_LED, PWM_White); // 4 correct work of interrpt
  //digitalWrite(UV_LED, HIGH);// 4 correct work of interrpt
  //digitalWrite(RED_LED, HIGH);// 4 correct work of interrpt
  Serial.begin(115200);
  Serial.setTimeout(100);
  //  pinMode(strobeInput,INPUT);
  //  attachInterrupt(strobeInput, Strobe_Input_Handler, RISING); // 4 ARDUINO
  attachInterrupt(digitalPinToInterrupt(strobeInput), Strobe_Input_Handler, RISING); // 4 Rpi Pico
  //  pinMode(strobeInput, INPUT_PULLUP); // 4 Rpi Pico pull_up must be after the attachinterrupt. It's a bug.
  pinMode(strobeInput, INPUT); // 4 Rpi Pico pull_up must be after the attachinterrupt. It's a bug.
  digitalWrite(solenoid_DIR, LOW);
  digitalWrite(solenoid_ON, LOW);
  motorsCalibration();
  //      while (!Serial) delay(10);
  ////
  //    Serial.println(F("Adafruit VL53L1X sensor demo"));
  Wire.begin(400000);
  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  //   Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  //    Serial.print(F("Timing budget (ms): "));
  //    Serial.println(vl53.getTimingBudget());

  /*
    vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
    vl.VL53L1X_SetInterruptPolarity(0);
    //  */
}

void motorsCalibration()
{
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
  }
  else {
    //    analogWrite(UV_LED, 0);
    //    analogWrite(RED_LED, PWM_Red);
    analogWrite(UV_LED, M[0][1]);
    analogWrite(RED_LED, M[1][1]);
    analogWrite(WHITE_LED, M[2][1]);
    analogWrite(IR_LED, M[3][1]);
  }

  counter += 1; // + синхр.
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
    Serial.println("UV has been changed, modes cache was refreshed");
    Serial.println(PWM_VAL);
  }

  if (cmd.substring(0, 2) == "WH") {
    PWM_VALH = cmd[2] - '0';
    PWM_VALL = cmd[3] - '0';
    PWM_VALlowest = cmd[4] - '0';
    PWM_VAL = (PWM_VALH * 100) + (PWM_VALL * 10) + (PWM_VALlowest * 1);
    PWM_White = PWM_VAL;
    modesCacheRefresh();
    analogWrite(WHITE_LED, PWM_White);
    Serial.println("WH has been changed, modes cache was refreshed");
    Serial.println(PWM_VAL);
  }

  if (cmd.substring(0, 2) == "RE") {
    PWM_VALH = cmd[2] - '0';
    PWM_VALL = cmd[3] - '0';
    PWM_VALlowest = cmd[4] - '0';
    PWM_VAL = (PWM_VALH * 100) + (PWM_VALL * 10) + (PWM_VALlowest * 1);
    PWM_Red = PWM_VAL;
    modesCacheRefresh();
    //    analogWrite(RED_LED, PWM_Red);
    Serial.println("RE has been changed, modes cache was refreshed");
    Serial.println(PWM_VAL);
  }

  if (cmd.substring(0, 2) == "FC") {
    actualFilter = cmd[3] - '0';
    filterChange(actualFilter);
  }

  if (cmd.substring(0, 5) == "DIST?") {
    distanceMeas();
  }

  if (cmd.substring(0, 1) == "M") {
    mode = cmd[1] - '0';
    Serial.println("mode has been changed");
    Serial.println(mode);
    if (mode == 1)
    {

      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M1[i][j];
    }
    if (mode == 2)
    {

      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M2[i][j];
    }
    if (mode == 3)
    {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M3[i][j];
    }
    if (mode == 4)
    {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M4[i][j];
    }
    if (mode == 5)
    {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M5[i][j];
    }
    if (mode == 6)
    {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M6[i][j];
    }
    if (mode == 7)
    {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M7[i][j];
    }
    if (mode == 0)
    {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M0[i][j];
    }
  }
}

void distanceMeas(void)
{
  int16_t distance;

  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(" mm");

    // data is read out, time for another reading!
    vl53.clearInterrupt();
  }
}

void filterChange(uint8_t actualFilter)
{
  digitalWrite(solenoid_ON, HIGH);
  delay(5);
  if (actualFilter == 0)
  {
    digitalWrite(solenoid_DIR, HIGH);
    delay(5);
  }
  else
  {
    digitalWrite(solenoid_DIR, LOW);
    delay(5);
  }
  digitalWrite(solenoid_ON, LOW);
}

void zoom(uint8_t dir)
{
  digitalWrite(nMotorsSleep, HIGH);
  uint32_t zoomCount = 0;
  if (dir == 1)
  {
    digitalWrite(dirPin_2, HIGH);
    while (analogRead(VAR_Y_pin) >= 767)
    {
      digitalWrite(stepPin_2, HIGH);
      delay(1);
      digitalWrite(stepPin_2, LOW);
      delay(1);
      zoomCount += 1;
      Serial.println(zoomCount);
    }
  }
  if (dir == 0)
  {
    digitalWrite(dirPin_2, LOW);
    while (analogRead(VAR_Y_pin) <= 256)
    {
      digitalWrite(stepPin_2, HIGH);
      delay(1);
      digitalWrite(stepPin_2, LOW);
      delay(1);
      zoomCount += 1;
      Serial.println(zoomCount);
    }
  }
  digitalWrite(nMotorsSleep, LOW);
}

void focus(uint8_t dir)
{
  digitalWrite(nMotorsSleep, HIGH);
  uint32_t focusCount = 0;
  if (dir == 1)
  {
    digitalWrite(dirPin_1, HIGH);
    while (analogRead(VAR_X_pin) >= 767)
    {
      digitalWrite(stepPin_1, HIGH);
      delay(1);
      digitalWrite(stepPin_1, LOW);
      delay(1);
      focusCount += 1;
      Serial.println(focusCount);
    }
  }
  if (dir == 0)
  {
    digitalWrite(dirPin_1, LOW);
    while (analogRead(VAR_X_pin) <= 256)
    {
      digitalWrite(stepPin_1, HIGH);
      delay(1);
      digitalWrite(stepPin_1, LOW);
      delay(1);
      focusCount += 1;
      Serial.println(focusCount);
    }
  }
  digitalWrite(nMotorsSleep, LOW);
}

void loop()
{
  VAR_X = analogRead(VAR_X_pin);
  VAR_Y = analogRead(VAR_Y_pin);

  if ((VAR_Y >= 767))
  {
    zoom(1);
  }
  if (VAR_Y <= 256)
  {
    zoom(0);
  }

  if ((VAR_X >= 767))
  {
    focus(1);
  }
  if (VAR_X <= 256)
  {
    focus(0);
  }

//  Serial.print("X = ");
//  Serial.print(VAR_X);
//  Serial.print("\t Y = ");
//  Serial.println(VAR_Y);

//  delay(20);
//  Serial.println(counter);
//  digitalWrite(3, HIGH);
//  filterChange(0);
//  delay(500);
//  digitalWrite(3, LOW);
//  filterChange(1);
//  delay(500);
  if (Serial.available())
  {
    waiting_4_command();
  }

}
