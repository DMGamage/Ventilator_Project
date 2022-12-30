#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <sSense-BMx280I2C.h>
#include <Wire.h>

#define MCP4725_ADDR 0x60

#define V_BPM  12
#define KC_BPM 2800
// Pin diclareration

#define INC 4  // increase button
#define DECR 10  // decrease button
#define START 11 // Start 
#define SET_RESET 12  // Set Reset 
#define BPMpin 2
#define Buzzer 13

// Loop time declarations
#define FAST_LOOP 20 // 50 ms loop
#define SLOW_LOOP 200
#define RESP_LOOP 3000
#define SERIAL_SPEED  9600
// Variable declaration
int BPM = 0;
int PEEP = 5;
int PIP = 30;

float pressure_pre = 1027;
int voltage = 0;
int SetBPM = 45;
int SetPEEP = 5;
int SetPIP = 30;
boolean Set = true;
int STARTED = false;
int input_loc = 0; // define the location of curser  BPM , PEEP or PIP
int Inputs[] = {SetBPM, PEEP, PIP};
unsigned long Time1 = 0;
unsigned long Time2 = 0;
unsigned long Time3 = 0;
int counter = 0;
// BPM counter
int BPM_Counter = 0;
float T_BPM = 0;
float t1_bpm = 0;
int T_buzzer = 0;
boolean Mute = false;
boolean SetAlarm = false;
boolean PEEP_SET = false;
float temp(NAN), hum(NAN), pres(NAN);
int   Pressure[120];
float tempint(NAN), humint(NAN), presint(NAN);
int IndexP = 0;
int MaxP = 0;
int MinP = 0;
boolean BME_Presence = true;

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

  pinMode(INC, INPUT);
  pinMode(DECR, INPUT);
  pinMode(START, INPUT);
  pinMode(SET_RESET, INPUT);
  pinMode( Buzzer, OUTPUT);
  Serial.begin(9600);
  // LCD intializing
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("Ventilator");
  delay(2000);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Set SPEED:");
  lcd.print(SetBPM);
  lcd.setCursor(0, 1);
  lcd.print("PEEP>");
  lcd.print(PEEP);
  lcd.print("  PIP<");
  lcd.print(PIP);
  delay(1000);

  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);

  voltage = 0;
  // cmd to update the DAC
  Wire.write(voltage >> 4);       // the 8 most significant bits...
  Wire.write((voltage & 15) << 4); // the 4 least significant bits...
  Wire.endTransmission();
  Wire.begin();

  // pressure sensor
  for (int i = 0; i < 120; i++) // pressur array to hold all pressure values
    Pressure[i] = 0;


  Wire.begin();
  while (!ssenseBMx280.begin() && BME_Presence)
  {
    DebugPort.println("Could not find BME/BMP280 sensor!");
    delay(500);
    counter++;
    lcd.setCursor(0, 1);
    lcd.print("Intialize Sensors");
    if (counter > 20)
      BME_Presence = false;
  }

  if (BME_Presence)
  {
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
  }
  else
  {
    lcd.setCursor(0, 1);
    lcd.print("BME Sensor ERR");
  }
  //  time loop init
  Time1 = millis();
  Time2 = millis();
  Time3 = millis();
  //  time loop init
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
  {
    digitalWrite(Buzzer, HIGH);
    while (digitalRead(SET_RESET)) continue;
    digitalWrite(Buzzer, LOW);
    Set = ~Set;
    if(SetAlarm) Mute = true;
    button_press = true;

    input_loc++;  if (input_loc > 2)   input_loc = 0;
  }
  else if (digitalRead(INC))
  {
    digitalWrite(Buzzer, HIGH);
    while (digitalRead(INC)) continue;
    digitalWrite(Buzzer, LOW);
    button_press = true;
    if (!Set)
    { if (STARTED)  input_loc = 0;
      input_loc++;  if (input_loc > 2)   input_loc = 0;

    }
    else
    { if (STARTED)  input_loc = 0;
      Inputs[input_loc] = Inputs[input_loc] + 1;
    }
  }
  else if (digitalRead(DECR))
  {
    digitalWrite(Buzzer, HIGH);
    while (digitalRead(DECR)) continue;
    button_press = true;
    digitalWrite(Buzzer, LOW);
    if (!Set)
    {
      input_loc--;  if (input_loc < 0)   input_loc = 2;
      
      
      if (STARTED)  input_loc = 0;
    }
    else
    {
      if (STARTED)  input_loc = 0;
      Inputs[input_loc] = Inputs[input_loc] - 1;
    }
  }
  else if (digitalRead(START))
  {
    digitalWrite(Buzzer, HIGH);
    while (digitalRead(START)) continue;
    button_press = true;
    digitalWrite(Buzzer, LOW);
    STARTED = ~STARTED ;
    Mute = false;
    input_loc = 0;
  }
  if (button_press)
  {
    Display();
    button_press = false;
  }
}

void Display()
{

  if (Inputs[0] < 0) Inputs[0] = 0;
  if (Inputs[0] > 100) Inputs[0] = 100;
  if (Inputs[1] < 0)   Inputs[1] = 0;
  if (Inputs[1] > 25) Inputs[1] = 25;
  if (Inputs[2] < 10) Inputs[2] = 10;
  if (Inputs[2] > 50) Inputs[2] = 50;
  SetBPM = Inputs[0];
  SetPEEP = Inputs[1];
  SetPIP = Inputs[2];
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Set SPEED:");
  lcd.print(SetBPM);
  lcd.setCursor(0, 1);
  lcd.print("PEEP>");
  lcd.print(SetPEEP);
  lcd.print("  PIP<");
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
    if (BME_Presence)
    {
      BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
      BME280::PresUnit presUnit(BME280::PresUnit_Pa);
      ssenseBMx280.read(pres, temp, hum, tempUnit, presUnit);
      Pressure[IndexP] = int((pres - presint) * 0.010197162129779282);
      // Pressure Array
      IndexP = IndexP + 1;

      if (IndexP > 119)
      {
        IndexP = 0; // reset on overflow
      }

      Serial.print(Pressure[IndexP]);
      Serial.println(",");
    }
    else
      Serial.println("BME SENSOR ERROR");
  }


  if ((millis() - Time1) > SLOW_LOOP)
  {
    Time1 = millis();
    if (STARTED)
    {

      //client->print("Pressure: ");
      lcd.clear();
      lcd.setCursor(0, 0);
      if (PEEP_SET)
      {
        lcd.print("SPD|BPM|PIP|RH");
      }
      else
        lcd.print("SPD|BPM|PIP|PEP");
      lcd.setCursor(0, 1);
      BPM = 60 / T_BPM;
      lcd.print(SetBPM);
      lcd.setCursor(3, 1); lcd.print("|");
      lcd.print(BPM);

      lcd.setCursor(7, 1); lcd.print("|");

      if (BME_Presence)
      {

        lcd.print(PIP);
        lcd.setCursor(11, 1); lcd.print("|");

        if (PEEP_SET)

        {
          lcd.print(hum, 0);
        }
        else
          lcd.print(PEEP);
        delay(10);
      }
      else
      {
        lcd.print("NA");
        lcd.setCursor(11, 1); lcd.print("|"); lcd.print("NA");
        delay(10);
      }
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

    if (STARTED&&BME_Presence && Set && BPM > 10 && PIP > SetPIP && !Mute)
    {
      SetAlarm = true;
      lcd.setCursor(7, 1); lcd.print("|"); lcd.print("ERR");
      digitalWrite(Buzzer, HIGH);
      delay(50);
      digitalWrite(Buzzer, LOW);
      delay(50);
      digitalWrite(Buzzer, HIGH);
      delay(50);
      digitalWrite(Buzzer, LOW);
      delay(500);
    }
    if (STARTED&&BME_Presence && BPM > 10 && Set && PEEP < SetPEEP && !Mute && PIP > SetPEEP   ) //
    {
      SetAlarm = true;
      lcd.setCursor(11, 1); lcd.print("|"); lcd.print("ERR");
      digitalWrite(Buzzer, HIGH);
      delay(200);
      digitalWrite(Buzzer, LOW);
      delay(200);
      digitalWrite(Buzzer, HIGH);
      delay(200);
      digitalWrite(Buzzer, LOW);
      delay(500);
    }

    if (PEEP_SET) PEEP_SET = false;
    else
      PEEP_SET = true;
    Serial.print("PEEP_SET");
    Serial.println(PEEP_SET);

  }
}

void Motor_update()
{
  if (STARTED)
  {
    voltage = SetBPM * V_BPM + KC_BPM;
    voltage = min(voltage, 4095);

  }
  else
    voltage = 0;
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);
  Wire.write(voltage >> 4);       // the 8 most significant bits...
  Wire.write((voltage & 15) << 4); // the 4 least significant bits...
  Wire.endTransmission();
  Wire.begin();
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

  if (BPM_Counter >= 600)
  {
    T_BPM = 2000;
  }

}


