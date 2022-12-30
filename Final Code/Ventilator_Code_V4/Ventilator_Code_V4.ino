#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <sSense-BMx280I2C.h>
#include <Wire.h>

#define MCP4725_ADDR 0x60
// Pin diclareration

#define INC 4  // increase button
#define DECR 10  // decrease button
#define START 11 // Start 
#define SET_RESET 12  // Set Reset 
#define BPMpin 2

#define V_bpm  30
#define KC_bpm 2600

#define buzzer 13
// Motor Declareration
#define BRAKE 0
#define CW    1
#define CCW   2
#define CS_THRESHOLD 15   // Definition of safety current (Check: "1.3 Monster Shield Example").

//MOTOR 1
#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8

//MOTOR 2
#define MOTOR_A2_PIN 4
#define MOTOR_B2_PIN 9

#define PWM_MOTOR_1 5
#define PWM_MOTOR_2 6

//#define CURRENT_SEN_1 A2
//#define CURRENT_SEN_2 A3

#define EN_PIN_1 A0
#define EN_PIN_2 A1

#define MOTOR_1 0
#define MOTOR_2 1

#define MOTOR_FET 3
#define HALL_PIN 2
// Loop time declarations
#define FAST_LOOP 20 // 50 ms loop
#define SLOW_LOOP 200
#define RESP_LOOP 3000
#define SERIAL_SPEED  9600

#define initialVoltage =2800;
#define Kp =10;
# define KI=100;
// Variable declaration
int BPM = 10;
int PEEP = 5;
int PIP = 30;
float pressure_pre = 1027;



int SetBPM = 10;
int SetPEEP = 5;
int SetPIP = 30;
int Set = true;
int STARTED = false;
int input_loc = 0; // define the location of curser  BPM , PEEP or PIP
int Inputs[] = {BPM, PEEP, PIP};
unsigned long Time1 = 0;
unsigned long Time2 = 0;
unsigned long Time3 = 0;
int buzzer_state = LOW;
int voltage = 0;
// BPM counter
long int BPM_Counter = 0;
float T_BPM = 0;
unsigned long t1_bpm = 0;

float temp(NAN), hum(NAN), pres(NAN);
int   Pressure[120];
float tempint(NAN), humint(NAN), presint(NAN);
int IndexP = 0;
int MaxP = 0;
int MinP = 0;


// Motor Variables
volatile byte state = LOW;

short usSpeed = 150;  //default motor speed
unsigned short usMotor_Status = BRAKE;
void Forward();
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm);

BMx280I2C::Settings settings(
  BME280::OSR_X2,
  BME280::OSR_X1,
  BME280::OSR_X16,
  BME280::Mode_Normal,
  BME280:: StandbyTime_500us,
  BME280::Filter_16,
  BME280::SpiEnable_False,
  0x76 // I2C address. I2C specific.
);

BMx280I2C ssenseBMx280(settings);
LiquidCrystal_I2C lcd(0x27, 16, 2);
void setup()
{
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);

  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);

  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);

  pinMode(MOTOR_FET, OUTPUT); // motor output fet
  analogWrite(MOTOR_FET, 155);

  // pinMode(CURRENT_SEN_1, OUTPUT);
  // pinMode(CURRENT_SEN_2, OUTPUT);

  pinMode(INC, INPUT);
  pinMode(DECR, INPUT);
  pinMode(START, INPUT);
  pinMode(SET_RESET, INPUT);
  pinMode(buzzer, OUTPUT);
  Serial.begin(9600);
  // LCD intializing
  lcd.init();
  lcd.backlight();

  delay(2000);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Set BPM:");
  lcd.print(BPM);
  lcd.setCursor(0, 1);
  lcd.print("PEEP=");
  lcd.print(PEEP);
  lcd.print("  PIP=");
  lcd.print(PIP);
  delay(1000);

  // pressure sensor
  for (int i = 0; i < 120; i++) // pressur array to hold all pressure values
    Pressure[i] = 0;


  Wire.begin();
  while (!ssenseBMx280.begin())
  {
    DebugPort.println("Could not find BME/BMP280 sensor!");
    delay(1000);
  }

  switch (ssenseBMx280.chipModel())
  {
    case BME280::ChipModel_BME280:
      DebugPort.println("Found BME280 sensor! Humidity available.");
      break;
    case BME280::ChipModel_BMP280:
      DebugPort.println("Found BMP280 sensor! No Humidity available.");
      break;
    default:
      DebugPort.println("Found UNKNOWN sensor! Error!");
  }

  // Change some settings before using.
  settings.tempOSR = BME280::OSR_X4;

  ssenseBMx280.setSettings(settings);


  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  ssenseBMx280.read(presint, tempint, humint, tempUnit, presUnit);
  delay(1000);
  ssenseBMx280.read(presint, tempint, humint, tempUnit, presUnit);
  delay(1000);
  ssenseBMx280.read(presint, tempint, humint, tempUnit, presUnit);
  delay(1000);


  //  time loop init
  Time1 = millis();
  Time2 = millis();
  Time3 = millis();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  noInterrupts();           // disable all interrupts
  OCR1A = 625;           // compare match register 16MHz/256/2Hz 10ms time 62500/100
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
  // BPM
  attachInterrupt(digitalPinToInterrupt(BPMpin), BPM_Loop, FALLING);
}

void loop()
{

  Check_Input();
  Check_Sensors();
  Motor_update();


}

void Check_Input()
{
  int button_press = false;
  if (digitalRead(SET_RESET))
  { digitalWrite(buzzer, HIGH);
    while (digitalRead(SET_RESET)) continue;
    Set = ~Set;
    button_press = true;
    digitalWrite(buzzer, LOW);

    input_loc++;  if (input_loc > 2)   input_loc = 0;
  }
  else if (digitalRead(INC))
  {
    digitalWrite(buzzer, HIGH);
    while (digitalRead(INC)) continue;
    button_press = true;
    digitalWrite(buzzer, LOW);
    if (!Set)
    {
      input_loc++;  if (input_loc > 2)   input_loc = 0;
    }
    else
    {
      Inputs[input_loc] = Inputs[input_loc] + 1;
    }
  }
  else if (digitalRead(DECR))
  {
    digitalWrite(buzzer, HIGH);
    while (digitalRead(DECR)) continue;
    button_press = true;
    digitalWrite(buzzer, LOW);
    if (!Set)
    {
      input_loc--;  if (input_loc < 0)   input_loc = 2;
    }
    else
    {
      Inputs[input_loc] = Inputs[input_loc] - 1;
    }
  }
  else if (digitalRead(START))
  {
    digitalWrite(buzzer, HIGH);
    while (digitalRead(START)) continue;
    button_press = true;
    digitalWrite(buzzer, LOW);
    STARTED = ~STARTED ;
  }
  if (button_press)
  {
    Display();
    button_press = false;
  }
}

void Display()
{

  if (Inputs[0] < 10) Inputs[0] = 10;
  if (Inputs[0] > 50) Inputs[0] = 50;
  if (Inputs[1] < 0)   Inputs[1] = 0;
  if (Inputs[1] > 25) Inputs[1] = 25;
  if (Inputs[2] < 10) Inputs[2] = 10;
  if (Inputs[2] > 50) Inputs[2] = 50;
  SetBPM = Inputs[0];
  SetPEEP = Inputs[1];
  SetPIP = Inputs[2];
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Set BPM:");
  lcd.print(T_BPM);
  //Serial.println(T_BPM);
  lcd.setCursor(0, 1);
  lcd.print("PEEP=");
  lcd.print(SetPEEP);
  lcd.print("  PIP=");
  lcd.print(SetPIP);


}


void Check_Sensors()
{
  if (Time1 < 0) Time1 = 0;
  if (Time2 < 0) Time2 = 0;
  if (Time3 < 0) Time3 = 0;

  if ((millis() - Time2) > FAST_LOOP)
  {
    Time2 = millis();
    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);
    ssenseBMx280.read(pres, temp, hum, tempUnit, presUnit);

    // Pressure Array
    IndexP = IndexP + 1;

    if (IndexP > 120)
    {
      IndexP = 0; // reset on overflow
    }
    Pressure[IndexP] = int((pres - presint) * 0.010197162129779282);
    /*
      Serial.print("#");
      Serial.print(Pressure[IndexP]);
      Serial.println(",");
    */
  }


  if ((millis() - Time1) > SLOW_LOOP)
  {
    Time1 = millis();
    if (STARTED)
    {

      //client->print("Pressure: ");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("BPM|PEEP|PIP|RH");
      lcd.setCursor(0, 1);
      // T_BPM = 60000 / T_BPM;
      lcd.print(T_BPM);
      lcd.setCursor(3, 1); lcd.print("|");
      lcd.print(SetPEEP);
      lcd.setCursor(8, 1); lcd.print("|"); lcd.print(PIP);
      lcd.setCursor(12, 1); lcd.print("|"); lcd.print(hum, 0);
      delay(10);
    }
  }

  if ((millis() - Time3) > RESP_LOOP) // 6s loop
  {
    Time3 = millis();

    MaxP = Pressure[0];
    MinP = Pressure[0];
    for (int i = 0; i < 120; i++)
    {
      //  Serial.println(Pressure[i]);


      if ((Pressure[i] > MaxP)) MaxP = Pressure[i];
      if ((Pressure[i] < MinP) && Pressure[i] != 0 ) MinP = Pressure[i];

    }
    PIP = MaxP;
    PEEP = MinP;

  }
}

void Motor_update()
{
  if (STARTED)
  {
    Turn();
  }
  else
    Stop();

}
void Turn()
{
  //digitalWrite(EN_PIN_1, HIGH);  // if we use 30A monster boards
  // digitalWrite(EN_PIN_2, HIGH);
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);
  voltage = SetBPM * V_bpm + KC_bpm; // analog DAC motors
  voltage = min(voltage, 4095);
  // cmd to update the DAC
  Wire.write(voltage >> 4);       // the 8 most significant bits...
  Wire.write((voltage & 15) << 4); // the 4 least significant bits...
  Wire.endTransmission();

  usSpeed = 3.05 * SetBPM  + 25; // for PWM motors
  Forward();

}
void Stop()
{
  //Serial.println("Stop");
  usMotor_Status = BRAKE;
  motorGo(MOTOR_1, usMotor_Status, 0);
  motorGo(MOTOR_2, usMotor_Status, 0);
}
void Forward()
{
  /*
    Serial.println("Forward");
    usMotor_Status = CW;
    motorGo(MOTOR_1, usMotor_Status, usSpeed);
    motorGo(MOTOR_2, usMotor_Status, usSpeed);

  */


}


void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)         //Function that controls the variables: motor(0 ou 1), direction (cw ou ccw) e pwm (entra 0 e 255);
{
  if (motor == MOTOR_1)
  {
    if (direct == CW)
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if (direct == CCW)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);
    }

    analogWrite(PWM_MOTOR_1, pwm);
  }
  else if (motor == MOTOR_2)
  {
    if (direct == CW)
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if (direct == CCW)
    {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);
    }
    else
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);
    }

    analogWrite(PWM_MOTOR_2, pwm);
  }

}
void BPM_Loop()
{
  float temp_BPM = BPM_Counter * 0.01;

  if (temp_BPM <= 10 and temp_BPM >= 0.45)
    T_BPM = temp_BPM;

  BPM_Counter = 0;
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine

{
  BPM_Counter++;
}



