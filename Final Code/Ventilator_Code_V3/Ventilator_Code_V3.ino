#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <sSense-BMx280I2C.h>
#include <Wire.h>


// Pin diclareration

#define INC 4  // increase button
#define DECR 10  // decrease button
#define START 11 // Start 
#define SET_RESET 12  // Set Reset 
#define BPMpin 2


// Loop time declarations
#define FAST_LOOP 20 // 50 ms loop
#define SLOW_LOOP 200
#define RESP_LOOP 3000
#define SERIAL_SPEED  9600
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

// BPM counter
int BPM_Counter = 0;
unsigned long T_BPM = 0;
unsigned long t1_bpm = 0;

float temp(NAN), hum(NAN), pres(NAN);
int   Pressure[120];
float tempint(NAN), humint(NAN), presint(NAN);
int IndexP = 0;
int MaxP = 0;
int MinP = 0;

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
  Serial.begin(9600);
  // LCD intializing
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("Ventilator");
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

  // BPM
  attachInterrupt(digitalPinToInterrupt(BPMpin), BPM_Loop, RISING);
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
    while (digitalRead(SET_RESET)) continue;
    Set = ~Set;
    button_press = true;

    input_loc++;  if (input_loc > 2)   input_loc = 0;
  }
  else if (digitalRead(INC))
  {
    while (digitalRead(INC)) continue;
    button_press = true;
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
    while (digitalRead(DECR)) continue;
    button_press = true;
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
    while (digitalRead(START)) continue;
    button_press = true;
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
  lcd.print(SetBPM);
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
    Serial.print("#");
    Serial.print(Pressure[IndexP]);
    Serial.println(",");
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
      if (millis() - t1_bpm < 6000) // 300 =1000/3  and 6000 6s maximum tigger time
      {

        BPM = 60000 / T_BPM;
        lcd.print(BPM);
      }
      else
        lcd.print(0);
      lcd.setCursor(3, 1); lcd.print("|");
      lcd.print(PEEP);
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

}

void BPM_Loop()
{
  if (!BPM_Counter)  t1_bpm = millis();
  BPM_Counter++ ;
  if (BPM_Counter > 2)
  {
    BPM_Counter = 0;
    T_BPM = millis() - t1_bpm;

  }

}

