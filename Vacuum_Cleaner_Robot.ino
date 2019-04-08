#include <avr/eeprom.h>

//---------------------------------------------------------
// --------------------- LCD START ------------------------
//---------------------------------------------------------
const int RS = 12;
const int E = 11;
const int D4 = 5;
const int D5 = 4;
const int D6 = 3;
const int D7 = 2;

bool bFourBitMode = false;
char ReadSendState = -1;

unsigned char Battery_6[8] =
{
  0b00100,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

unsigned char Battery_5[8] =
{
  0b00100,
  0b11111,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

unsigned char Battery_4[8] =
{
  0b00100,
  0b11111,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

unsigned char Battery_3[8] =
{
  0b00100,
  0b11111,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

unsigned char Battery_2[8] =
{
  0b00100,
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111
};

unsigned char Battery_1[8] =
{
  0b00100,
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111
};

unsigned char Battery_0[8] =
{
  0b00100,
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111
};

void LcdSend(unsigned char Data)
{
  if (bFourBitMode)
  {
    digitalWrite(D4, (Data >> 4) & 0x01);
    digitalWrite(D5, (Data >> 5) & 0x01);
    digitalWrite(D6, (Data >> 6) & 0x01);
    digitalWrite(D7, (Data >> 7) & 0x01);
    
    delayMicroseconds(10);
    digitalWrite(E, HIGH);
    delayMicroseconds(10);
    digitalWrite(E, LOW);
    delayMicroseconds(100);
  }
  
  digitalWrite(D4, (Data >> 0) & 0x01);
  digitalWrite(D5, (Data >> 1) & 0x01);
  digitalWrite(D6, (Data >> 2) & 0x01);
  digitalWrite(D7, (Data >> 3) & 0x01);
  
  delayMicroseconds(10);
  digitalWrite(E, HIGH);
  delayMicroseconds(10);
  digitalWrite(E, LOW);
  delayMicroseconds(100);
}

void LcdCommand(unsigned char Command)
{
  if (ReadSendState != LOW)
  {
    ReadSendState = LOW;
    digitalWrite(RS, LOW);
  }
  
  LcdSend(Command);
  if (Command == 0x01) delayMicroseconds(2000);// Clear command takes more time
}

void LcdWrite(int Letter)
{
  if (ReadSendState != HIGH)
  {
    ReadSendState = HIGH;
    digitalWrite(RS, HIGH);
  }
  
  LcdSend(Letter);
}

void LcdWrite(const char* Text)
{
  if (ReadSendState != HIGH)
  {
    ReadSendState = HIGH;
    digitalWrite(RS, HIGH);
  }
  
  for (; *Text != 0; Text++)
  {
    char Letter = *Text;
    LcdSend(Letter);
  }
}

void LcdInit(bool bFirstInit)
{
  if (bFirstInit)
  {
    // Give it time to power up
    delayMicroseconds(15000);
    
    pinMode(RS, OUTPUT);
    pinMode(E, OUTPUT);
    pinMode(D4, OUTPUT);
    pinMode(D5, OUTPUT);
    pinMode(D6, OUTPUT);
    pinMode(D7, OUTPUT);
  }
  
  // Start
  bFourBitMode = false;
  
  LcdCommand(0x03);
  
  delayMicroseconds(4000);
  
  LcdCommand(0x03);
  LcdCommand(0x03);
  LcdCommand(0x02);
  
  bFourBitMode = true;
  
  LcdCommand(0x28);
  LcdCommand(0x0C);
  LcdCommand(0x01);// Clear
  LcdCommand(0x06);
  
  LcdCreateChar(0, Battery_0);
  LcdCreateChar(1, Battery_1);
  LcdCreateChar(2, Battery_2);
  LcdCreateChar(3, Battery_3);
  LcdCreateChar(4, Battery_4);
  LcdCreateChar(5, Battery_5);
  LcdCreateChar(6, Battery_6);
}

void LcdSetCursor(unsigned char Column, unsigned char Row)
{
  LcdCommand(0x80 | (Column + (Row != 0 ? 0x40 : 0x00)));
}

void LcdCreateChar(unsigned char Location, unsigned char SpecialChar[8])
{
  LcdCommand(0x40 | (Location << 3));
  
  for (unsigned int i = 0; i < 8; i++)
    LcdWrite(SpecialChar[i]);
}

//---------------------------------------------------------
// --------------------- LCD END --------------------------
//---------------------------------------------------------

//---------------------------------------------------------
// --------------------- KEYPAD START ---------------------
//---------------------------------------------------------

const int DOD = 50;
const int DOC = 51;
const int DOB = 52;
const int DOA = 53;

volatile char PendingKey = NULL;

char ReadKeyPad()
{
  char Key = PendingKey;
  PendingKey = NULL;
  return Key;
}

void KeyPadInterrupt()
{
  if (PendingKey != NULL) return;
  
  char ReturnValue = 0;
  
  ReturnValue |= digitalRead(DOA) << 0;
  ReturnValue |= digitalRead(DOB) << 1;
  ReturnValue |= digitalRead(DOC) << 2;
  ReturnValue |= digitalRead(DOD) << 3;
  
  static char Keys[4][3] = {{'1', '2', '3'},
                            {'4', '5', '6'},
                            {'7', '8', '9'},
                            {'*', '0', '#'}};
  
  PendingKey = Keys[ReturnValue % 4][ReturnValue / 4];
}

//---------------------------------------------------------
// --------------------- KEYPAD END -----------------------
//---------------------------------------------------------

char CurrentServoDegrees = -1;

void MoveServo(char Degrees)
{
  if (Degrees != CurrentServoDegrees)
  {
    OCR5A = (unsigned short)((0.075f + (float)Degrees / 1800.0f) * (float)ICR5);
    // Duty Cycle = 1 - 2 [ms]
    // PWM Period = 20 [ms]
    delay(1000);
    CurrentServoDegrees = Degrees;
  }
}

int MaxBatteryEverSeen;
int MaxDustEverSeen;

enum Direction
{
  Left,
  Right,
  Reverse,
  Advance,
  Brake
};

char RobotDirection = Brake;
bool bRobotEnabled = false;
float fOneBattery;
const float fWheelsVoltage = 4.5f;

void ChangeRobotMovement(char MoveTo)
{
  if (bRobotEnabled && MoveTo != RobotDirection)
  {
    if (MoveTo == Advance)
    {
      digitalWrite(31, HIGH);
      digitalWrite(32, LOW);
      digitalWrite(33, HIGH);
      digitalWrite(34, LOW);
    }
    else if (MoveTo == Brake)
    {
      digitalWrite(31, LOW);
      digitalWrite(32, LOW);
      digitalWrite(33, LOW);
      digitalWrite(34, LOW);
      analogWrite(8, 0 * 255 / 100);
    }
    else if (MoveTo == Reverse)
    {
      digitalWrite(31, LOW);
      digitalWrite(32, HIGH);
      digitalWrite(33, LOW);
      digitalWrite(34, HIGH);
    }
    else if (MoveTo == Right)
    {
      digitalWrite(31, LOW);
      digitalWrite(32, HIGH);
      digitalWrite(33, HIGH);
      digitalWrite(34, LOW);
    }
    else if (MoveTo == Left)
    {
      digitalWrite(31, HIGH);
      digitalWrite(32, LOW);
      digitalWrite(33, LOW);
      digitalWrite(34, HIGH);
    }
    
    if (MoveTo != Brake) analogWrite(8, (fWheelsVoltage / 3.0f) / fOneBattery * 255.0f);
    
    RobotDirection = MoveTo;
  }
}

unsigned int GetRange()
{
  digitalWrite(22, HIGH);
  delayMicroseconds(10);
  digitalWrite(22, LOW);
  unsigned long usDuration = pulseIn(23, HIGH);
  if (usDuration < 140000) delayMicroseconds(140000 - usDuration);// Wait at least 140ms each cycle
  return (usDuration / 58);
}

void setup()
{
  analogWrite(6, 0 * 255 / 100);
  analogWrite(8, 0 * 255 / 100);
  
  LcdInit(true);
  
  MaxBatteryEverSeen = eeprom_read_word((uint16_t*)0x0000);
  MaxDustEverSeen = eeprom_read_word((uint16_t*)0x0002);
  
  TCCR4B = (TCCR4B & ~(1 << CS41)) | (1 << CS40) | (1 << CS42);
  
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(34, OUTPUT);
  
  digitalWrite(31, LOW);
  digitalWrite(32, LOW);
  digitalWrite(33, LOW);
  digitalWrite(34, LOW);
  
  pinMode(22, OUTPUT);
  pinMode(23, INPUT);
  
  pinMode(10, OUTPUT);
  
  pinMode(DOD, INPUT);
  pinMode(DOC, INPUT);
  pinMode(DOB, INPUT);
  pinMode(DOA, INPUT);
  
  pinMode(18, INPUT);
  // INT.5 -> Pin 18
  attachInterrupt(5, KeyPadInterrupt, RISING);
  
  pinMode(46, OUTPUT);
  
  // None-inverted PWM on pin 46, Prescaler 8
  TCCR5A = (1 << COM5A1);
  TCCR5B = (1 << WGM53) | (1 << CS51);
  ICR5 = 20000;// 50Hz -> TOP=Fclk/(2*Prescaler*DesiredFrequency)=16M/(2*8*50)=20000
  MoveServo(0);// Middle
}

void loop()
{
  static unsigned long OneSecondTimer = 0;
  
  unsigned long CurrentTime = millis();
  
  if (CurrentTime >= OneSecondTimer)
  {
    OneSecondTimer = CurrentTime + 1000;// Run this 'IF Block' every 1 second
    
    // Battery range: 2.75V-4.2V
    const int BatteryMin = 2.75 * 1023 / 5;
    
    int CurrentBattery = analogRead(A0);//0-1023
    
    if (CurrentBattery > MaxBatteryEverSeen)
    {
      eeprom_write_word((uint16_t*)0x0000, CurrentBattery);
      MaxBatteryEverSeen = CurrentBattery;
    }
    
    fOneBattery = float(CurrentBattery) / 1023.0f * 5.0f;
    
    if (CurrentBattery > MaxBatteryEverSeen) CurrentBattery = 100;
    else if (CurrentBattery < BatteryMin) CurrentBattery = 0;
    else CurrentBattery = (CurrentBattery - BatteryMin) * 100 / (MaxBatteryEverSeen - BatteryMin);
    
    LcdInit(false);
    
    digitalWrite(10, HIGH);
    delayMicroseconds(280);// Wait for the dust sensor to sample
    int CurrentDust = analogRead(A1);
    digitalWrite(10, LOW);
  
    if (CurrentDust > MaxDustEverSeen)
    {
      eeprom_write_word((uint16_t*)0x0002, CurrentDust);
      MaxDustEverSeen = CurrentDust;
    }
    
    float DustSensor = float(CurrentDust) * 100.0f / float(MaxDustEverSeen);
    
    char KeyPressed = ReadKeyPad();
    
    static unsigned char DustDetectedCounter = 0;
    
    if (KeyPressed == '1') bRobotEnabled = true;
    else if (KeyPressed == '*')
    {
      ChangeRobotMovement(Brake);
      bRobotEnabled = false;
      DustDetectedCounter = 0;
      analogWrite(6, 0 * 255 / 100);
    }
    
    LcdSetCursor(0, 1);
    
    if (!bRobotEnabled) LcdWrite("Press 1 to start");
    else
    {
      if (RobotDirection != Brake) analogWrite(8, (fWheelsVoltage / 3.0f) / fOneBattery * 255.0f);
      
      if (DustSensor > 0.0f)
      {
        DustDetectedCounter = 15;
        analogWrite(6, 55 * 255 / 100);
      }
      else if (DustDetectedCounter != 0) DustDetectedCounter--;
      else analogWrite(6, 45 * 255 / 100);
      
      LcdWrite("Press * to stop");
    }
    
    LcdSetCursor(0, 0);
    LcdWrite(("Dust: " + String(DustSensor) + '%').c_str());
    
    LcdSetCursor(15, 0);
    LcdWrite(int(float(CurrentBattery) / 100.0f * 6.0f + 0.5f));
  }
  
  static unsigned long TimeOutTimer = 0;
  const unsigned int RangeToEvade = 30;
  
  if (CurrentTime < TimeOutTimer && GetRange() > RangeToEvade) ChangeRobotMovement(Advance);
  else
  {
    char EvadeMovement = Brake;
    ChangeRobotMovement(Brake);
    
    // Either check right or check left to decide:
    randomSeed(millis());
    
    if (random(2) == 0)
    {
      // Check left
      MoveServo(90);
      if (GetRange() > RangeToEvade) EvadeMovement = Left;// Evade left then
      else
      {
        // Check right
        MoveServo(-90);
        if (GetRange() > RangeToEvade) EvadeMovement = Right;// Evade right then
      }
    }
    else
    {
      // Check right
      MoveServo(-90);
      if (GetRange() > RangeToEvade) EvadeMovement = Right;// Evade right then
      else
      {
        // Check left
        MoveServo(90);
        if (GetRange() > RangeToEvade) EvadeMovement = Left;// Evade left then
      }
    }
    
    if (EvadeMovement == Brake)
    {
      // Nothing works so we will randomly decide
      randomSeed(millis());
      EvadeMovement = random(3);// 0-2
    }
    
    MoveServo(0);
    
    unsigned long EndTime;
    
    for (CurrentTime = millis(), EndTime = CurrentTime + 5000; bRobotEnabled && CurrentTime < EndTime; CurrentTime = millis())
    {
      ChangeRobotMovement(EvadeMovement);
      delay(500);
      
      if (GetRange() > RangeToEvade)
      {
        ChangeRobotMovement(Brake);
        MoveServo(-22);
        
        if (GetRange() > RangeToEvade)
        {
          MoveServo(22);
          
          if (GetRange() > RangeToEvade) break;
        }
      }
    }
    
    MoveServo(0);
    
    TimeOutTimer = CurrentTime + 10000;// After 10 seconds
  }
}
