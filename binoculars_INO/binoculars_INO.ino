//#include <Wire.h>
//#include "Adafruit_VL53L1X.h"
//#define IRQ_PIN 3
//#define XSHUT_PIN 28
//Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

String cmd, CMDcur;
uint8_t programNumber;

volatile int counter = 0;

uint8_t strobeInput = 2;
uint8_t UV_LED = 8;
uint8_t WHITE_LED = 6;
uint8_t RED_LED = 7;

uint8_t VAR_X_pin = 26;
uint8_t VAR_Y_pin = 27;

uint8_t nsleep = 9;

uint8_t nFLT1 = 10;
uint8_t nEnl = 11;
uint8_t stepPin_1 = 12;
uint8_t dirPin_1 = 13;
uint8_t m0_1 = 14;
uint8_t m1_1 = 15;

uint8_t nFLT2 = 21;
uint8_t nEn2 = 20;
uint8_t stepPin_2 = 19;
uint8_t dirPin_2 = 18;
uint8_t m0_2 = 17;
uint8_t m1_2 = 16;

volatile uint8_t PWM_White = 1;
volatile uint8_t PWM_UV = 1;
volatile uint8_t PWM_Red = 1;

uint8_t mode;

uint16_t VAR_X = 0;
uint16_t VAR_Y = 0;

volatile uint8_t M[3][2];
volatile uint8_t M0[3][2]
{
  {0, 0},
  {0, 0},
  {0, 0}
};
volatile uint8_t M1[3][2];
volatile uint8_t M2[3][2];
volatile uint8_t M3[3][2];
volatile uint8_t M4[3][2];
volatile uint8_t M5[3][2];
volatile uint8_t M6[3][2];


void modesCacheRefresh()
{
  Serial.println("cache refreshing");
  M1[0][0] = PWM_UV;
  M1[0][1] = 0;
  M1[1][0] = 0;
  M1[1][1] = 0;
  M1[2][0] = PWM_White;
  M1[2][1] = PWM_White;

  M2[0][0] = 0;
  M2[0][1] = 0;
  M2[1][0] = PWM_Red;
  M2[1][1] = 0;
  M2[2][0] = PWM_White;
  M2[2][1] = PWM_White;

  M3[0][0] = PWM_UV;
  M3[0][1] = 0;
  M3[1][0] = PWM_Red;
  M3[1][1] = 0;
  M3[2][0] = PWM_White;
  M3[2][1] = PWM_White;

  M4[0][0] = PWM_UV;          //oxygenation IR LEDs must be mounted instead of UV LEDs.
  M4[0][1] = 0;
  M4[1][0] = 0;
  M4[1][1] = PWM_Red;
  M4[2][0] = 0;
  M4[2][1] = 0;

  M5[0][0] = 0;             // ICG mode IR LEDs must be mounted instead of White LEDs.
  M5[0][1] = 0;
  M5[1][0] = 0;
  M5[1][1] = 0;
  M5[2][0] = PWM_White;
  M5[2][1] = 0;

  M6[0][0] = PWM_UV;          //Sequental stroboscope of red and UV LEDs.
  M6[0][1] = 0;
  M6[1][0] = 0;
  M6[1][1] = PWM_Red;
  M6[2][0] = PWM_White;
  M6[2][1] = PWM_White;
}

void setup()
{
  M1[0][0] = PWM_UV;
  M1[0][1] = 0;
  M1[1][0] = 0;
  M1[1][1] = 0;
  M1[2][0] = PWM_White;
  M1[2][1] = PWM_White;

  M2[0][0] = 0;
  M2[0][1] = 0;
  M2[1][0] = PWM_Red;
  M2[1][1] = 0;
  M2[2][0] = PWM_White;
  M2[2][1] = PWM_White;

  M3[0][0] = PWM_UV;
  M3[0][1] = 0;
  M3[1][0] = PWM_Red;
  M3[1][1] = 0;
  M3[2][0] = PWM_White;
  M3[2][1] = PWM_White;

  M4[0][0] = PWM_UV;          //oxygenation IR LEDs must be mounted instead of UV LEDs.
  M4[0][1] = 0;
  M4[1][0] = 0;
  M4[1][1] = PWM_Red;
  M4[2][0] = 0;
  M4[2][1] = 0;

  M5[0][0] = 0;             // ICG mode IR LEDs must be mounted instead of White LEDs.
  M5[0][1] = 0;
  M5[1][0] = 0;
  M5[1][1] = 0;
  M5[2][0] = PWM_White;
  M5[2][1] = 0;

  M6[0][0] = PWM_UV;          //Sequental stroboscope of red and UV LEDs.
  M6[0][1] = 0;
  M6[1][0] = 0;
  M6[1][1] = PWM_Red;
  M6[2][0] = PWM_White;
  M6[2][1] = PWM_White;

  pinMode(stepPin_1, OUTPUT);
  pinMode(dirPin_1, OUTPUT);
  pinMode(stepPin_2, OUTPUT);
  pinMode(dirPin_2, OUTPUT);
  pinMode(nsleep, OUTPUT);
  pinMode(nEnl, OUTPUT);
  pinMode(nEn2, OUTPUT);
  pinMode(nFLT1, INPUT);
  pinMode(nFLT2, INPUT);
  pinMode(m0_1, OUTPUT);
  pinMode(m1_1, OUTPUT);
  pinMode(m0_2, OUTPUT);
  pinMode(m1_2, OUTPUT);

  //  pinMode(strpbeInput, INPUT_PULLUP); /// Our camera strobe in HIGH - Acquiring, LOW - not acquiring
  pinMode(UV_LED, OUTPUT);// UV LED
  pinMode(RED_LED, OUTPUT);// UV LED
  pinMode(WHITE_LED, OUTPUT);// White LED
  pinMode(3, OUTPUT);// For migalka test

  analogWrite(WHITE_LED, PWM_White);
  analogWrite(UV_LED, PWM_White); // 4 correct work of interrpt
  analogWrite(RED_LED, PWM_White); // 4 correct work of interrpt
  //digitalWrite(UV_LED, HIGH);// 4 correct work of interrpt
  //digitalWrite(RED_LED, HIGH);// 4 correct work of interrpt
  Serial.begin(115200);
  Serial.setTimeout(100);
  //  pinMode(strobeInput,INPUT);
  //  attachInterrupt(strobeInput, Strobe_Input_Handler, RISING); // 4 ARDUINO
  attachInterrupt(digitalPinToInterrupt(strobeInput), Strobe_Input_Handler, RISING); // 4 Rpi Pico
  pinMode(strobeInput, INPUT_PULLUP); // 4 Rpi Pico pull_up must be after the attachinterrupt. It's a bug.
  //    pinMode(strobeInput,INPUT); // 4 Rpi Pico pull_up must be after the attachinterrupt. It's a bug.
  //  while (!Serial) delay(10);

  //  Serial.println(F("Adafruit VL53L1X sensor demo"));
  //  Wire.begin(400000);
  //  if (! vl53.begin(0x29, &Wire)) {
  //    Serial.print(F("Error on init of VL sensor: "));
  //    Serial.println(vl53.vl_status);
  //    while (1)       delay(10);
  //  }
  //  Serial.println(F("VL53L1X sensor OK!"));
  //
  //  Serial.print(F("Sensor ID: 0x"));
  //  Serial.println(vl53.sensorID(), HEX);
  //
  //  if (! vl53.startRanging()) {
  //    Serial.print(F("Couldn't start ranging: "));
  //    Serial.println(vl53.vl_status);
  //    while (1)       delay(10);
  //  }
  //  Serial.println(F("Ranging started"));
  //
  //  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  //  vl53.setTimingBudget(50);
  //  Serial.print(F("Timing budget (ms): "));
  //  Serial.println(vl53.getTimingBudget());
  //
  //  /*
  //  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  //  vl.VL53L1X_SetInterruptPolarity(0);
  //  */
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
    //    analogWrite(IR_LED, M[3][0]);
  }
  else {
    //    analogWrite(UV_LED, 0);
    //    analogWrite(RED_LED, PWM_Red);
    analogWrite(UV_LED, M[0][1]);
    analogWrite(RED_LED, M[1][1]);
    analogWrite(WHITE_LED, M[2][1]);
    //    analogWrite(IR_LED, M[3][1]);
  }

  counter += 1; // + синхр.
}

void waiting_4_command() {
  int PWM_VAL, PWM_VALH, PWM_VALL, PWM_VALlowest;
  cmd = "";
  if (Serial.available()) {
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

  if (cmd.substring(0, 1) == "M") {
    mode = cmd[1] - '0';
    Serial.println("mode has been changed");
    Serial.println(mode);
    if (mode == 1)
    {

      for (uint8_t i = 0; i < 3; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M1[i][j];
    }
    if (mode == 2)
    {

      for (uint8_t i = 0; i < 3; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M2[i][j];
    }
    if (mode == 3)
    {
      for (uint8_t i = 0; i < 3; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M3[i][j];
    }
    if (mode == 4)
    {
      for (uint8_t i = 0; i < 3; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M4[i][j];
    }
    if (mode == 5)
    {
      for (uint8_t i = 0; i < 3; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M5[i][j];
    }
        if (mode == 6)
    {
      for (uint8_t i = 0; i < 3; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M6[i][j];
    }
    if (mode == 0)
    {
      for (uint8_t i = 0; i < 3; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M0[i][j];
    }
  }
}

void loop()
{
  VAR_X = analogRead(VAR_X_pin);
  VAR_Y = analogRead(VAR_Y_pin);

  Serial.print("X = ");
  Serial.print(VAR_X);
  Serial.print("\t Y = ");
  Serial.println(VAR_Y);
  
//  Serial.println("X= ");
//  Serial.print(VAR_X);
////  Serial.print(" ");
////  Serial.println();
////  delay(10);
//  Serial.println("Y= ");
//  Serial.print(VAR_Y);
////  Serial.print(" ");
//  Serial.println();
//  delay(20);
    Serial.println(counter);
    digitalWrite(3, HIGH);
    delay(500);
    digitalWrite(3, LOW);
    delay(500);
    waiting_4_command();
  digitalWrite(nEnl, LOW);
  digitalWrite(dirPin_1, HIGH);
  digitalWrite(m1_1, LOW);
  digitalWrite(m0_1, LOW);

  digitalWrite(nEn2, LOW);
  digitalWrite(dirPin_2, LOW);
  digitalWrite(m1_2, LOW);
  digitalWrite(m0_2, LOW);

  digitalWrite(nsleep, HIGH);
    for (int x = 0; x < 200; x++) {
      digitalWrite(stepPin_1, HIGH);
      digitalWrite(stepPin_2, HIGH);
      delay(2);  // ms (Note : 1000ms = 1sec)
      digitalWrite(stepPin_1, LOW);
      digitalWrite(stepPin_2, LOW);
      delay(2); // ms (Note : 1000ms = 1sec)
//      Serial.println(x);
    }
}
