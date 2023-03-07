String cmd, CMDcur;
int programNumber;
//int lamp = 13;
volatile int counter = 0;
//volatile int lampCounter = 0;
uint8_t PWM_White = 2;
uint8_t PWM_UV = 200;
uint8_t PWM_Red = 200;
uint8_t UV_LED = 5;
uint8_t WHITE_LED = 6;
uint8_t RED_LED = 4;
uint8_t VAR_X_pin = 26;
uint8_t VAR_Y_pin = 27;
uint16_t VAR_X = 0;
uint16_t VAR_Y = 0;

void setup()
{
//  pinMode(2, INPUT_PULLUP); /// Our camera strobe in HIGH - Acquiring, LOW - not acquiring
  pinMode(UV_LED, OUTPUT);// UV LED
  pinMode(RED_LED, OUTPUT);// UV LED
  pinMode(WHITE_LED, OUTPUT);// White LED
//  pinMode(13, OUTPUT); ///LAMP !
  analogWrite(WHITE_LED, PWM_White);
//    analogWrite(UV_LED, PWM_White); // 4 correct work of interrpt
digitalWrite(UV_LED, LOW);
  Serial.begin(115200);
  Serial.setTimeout(100);
  pinMode(2,INPUT);
  attachInterrupt(0, Strob_Input_Handler, RISING); // 4 ARDUINO
//attachInterrupt(2, Strob_Input_Handler, HIGH); // 4 Rpi Pico
//  pinMode(2,INPUT); // 4 Rpi Pico pull_up must be after the attachinterrupt. It's a bug.
  
//  pinMode(2,INPUT_PULLUP); // 4 Rpi Pico pull_up must be after the attachinterrupt. It's a bug.
  while (!Serial) {
    ;
  }
}


void Strob_Input_Handler() { 
  if (counter == 2) {
    counter = 0;
    }
     if (counter == 1) {
//    analogWrite(UV_LED, PWM_UV);
digitalWrite(UV_LED, HIGH);
  }
  else {
//    analogWrite(UV_LED, 0);
    digitalWrite(UV_LED, LOW);
  }
  counter +=1;  // + синхр.
//  delay(10);
//  lampCounter += 1;
  
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
        Serial.println("UV has been changed");
        Serial.println(PWM_VAL);
  }

  if (cmd.substring(0, 2) == "WH") {
    PWM_VALH = cmd[2] - '0';
    PWM_VALL = cmd[3] - '0';
    PWM_VALlowest = cmd[4] - '0';
        PWM_VAL = (PWM_VALH * 100) + (PWM_VALL * 10) + (PWM_VALlowest * 1);
        PWM_White = PWM_VAL;
    analogWrite(WHITE_LED, PWM_White);
    Serial.println("WH has been changed");
    Serial.println(PWM_VAL);
  }
}

void loop()
{
  VAR_X = analogRead(VAR_X_pin);
  VAR_Y = analogRead(VAR_Y_pin);
  Serial.println(VAR_X);
  Serial.println(VAR_Y);
//  waiting_4_command();
}
