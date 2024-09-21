// Inlcude Servo,SPI,Wire,Display,DAC,MIDIUSB,EEPROM
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP4728.h>
#include "MIDIUSB.h"
#include "EEPROM.h"
#include "graphics.h"


// Calculation with Min,Max etc.
#define NEWCALC 1

//#define FRQ400KHZ 1
// Analog-Out aktivieren
// Voltage-Difference to lower DAC-interface-time
#define VOLTDIF 1 
#define MILLIPLUS 50

// Initialize Serial
//#define SSER 1

// Show Min/Max-Servo-Voltage-Calculations at the start
//#define SERCALC 1

// throw minor Info on serial
//#define SER1 1

// throw additional Serial-Debug-Info
//#define SER2 1

// Line of EEPROM-Messages
#define FZEIL 28

// Position of Activity-Circles
#define YACT 55
#define RACT 5
#define XACT 15
#define X1ACT 6

// MINMS,MAXMS -> times for a standard servo for maximum angles
#define MINMS 530
#define MAXMS 2400
// Same as Float
#define FMINMS 530.0
#define FMAXMS 2400.0

// Keypress-times
#define EXITPRESS 2000
#define LONGPRESS 500
#define REPEATPRESS 100
#define DEZIPRESS 150
#define PRELLO 50

// Display-Definitions
#define OLED_RESET -1
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// Keyboard-Ports
#define KEYLEFT 4
#define KEYUP 5
#define KEYDOWN 6
#define KEYRIGHT 7

// Internal Key-Numbers
#define KeyLeft 1
#define KeyUp 2
#define KeyDown 3
#define KeyRight 4

// Servo-Ports
#define PSERVO1 10
#define PSERVO2 14
#define PSERVO3 15
#define PSERVO4 16

// Maximum channels
#define MAXCHANNELS 8
// Data for EEPROM-size-calcs
#define MAXPAR 4

#define LENVERSION 16
#define LENDATA MAXCHANNELS*MAXPAR

// Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DAC
Adafruit_MCP4728 mcp;


// Servo-definitions
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;

// Global variables


// Save old keypresses
uint8_t oldKey[4];
//Key-Ports
uint8_t keyPort[4] = { KEYLEFT, KEYUP, KEYDOWN, KEYRIGHT };

//Which keys can take Longpresses
uint8_t longMask[4] = { true, true, true, true };
//Which Keys can take Repeat-Presses
uint8_t repeatMask[4] = { false, false, false, false };
//Which keys can take Exit-Presses
uint8_t exitMask[4] = { false, false, false, false };
//Which keys can take Dezi-fastklick
uint8_t deziMask[4] = {false,true,true,false };

uint8_t ready;
uint8_t refresh;
uint32_t timer, diffTime;
uint32_t deziTimer;
uint8_t i;

int8_t actPort, oldPort;

//Actual Midi-Config
uint8_t midiController[MAXCHANNELS];
uint8_t midiChannel[MAXCHANNELS];
uint8_t midiMin[MAXCHANNELS];
uint8_t midiMax[MAXCHANNELS];
uint8_t midiInvert[MAXCHANNELS];

//Servo-Positions and Voltage
uint16_t servoPos[4];
uint16_t oldservoPos[4];
uint16_t voltage[4];
uint16_t oldvoltage[4];

uint32_t milliSave,milliSave2;
uint8_t milliflag;

//Input-Mode
int8_t inputMode;


uint8_t flag;
char s[80];


/* float CalcVal(float val,uint8_t min,uint8_t max)
  Calculate the value reduced to Min and Max
  Val : 0-100
  min, max : 0-100
  Output: 0-100
*/

/* CalcMS(float val,uint8_t min,uint8_t max)
  Calculates the Milliseconds for Servo
  val: 0-1, min, max 0-100
*/
uint16_t CalcMS(float val,uint8_t min,uint8_t max)
{
  float minf,maxf,diff;
  float minf2,maxf2,diff2;
  float p1,p2;
  float px;
  minf=FMINMS; maxf=FMAXMS;
  diff=maxf-minf;
  minf2=((float)min)*0.01*diff;
  maxf2=((float)max)*0.01*diff;
  diff2=maxf2-minf2;
  px=minf+minf2+diff2*val;
  return((uint16_t)px);
}

/* CalcVOLT(float val,uint8_t min,uint8_t max)
  Calculates the Voltage for DAC
  val: 0-127, min, max 0-100
*/
uint16_t CalcVOLT(float val,uint8_t min,uint8_t max)
{
  float minf,maxf,diff;
  float p1,p2;
  float px;
  minf=0.0; maxf=4095.0;
  diff=maxf-minf;
  p1=(float)min; p2=(float)max;
  p1=p1*0.01; p2=p2*0.01;
  px=val*diff*(p2-p1);
  if(p2>p1)
    px=px+minf;
  else
    px=maxf+px;
  return((uint16_t)px);
}

/* int8_t scankey(uint8_t keyNr)
    Scans the key number 'keyNr' for normal presses
    and long presses
*/
int8_t scanKey(uint8_t keyNr)
{
  int8_t keyPressed1;
  // Button 'keyNr' pressed?
  keyPressed1 = -1;
  if (!digitalRead(keyPort[keyNr]))
  {
    // Key pressed, newly pressed?
    if (!oldKey[keyNr])
    {
      // Yes, newly pressed
      timer = millis();
      diffTime = 0;
      oldKey[keyNr] = true;
      if ((!longMask[keyNr]) && (!repeatMask[keyNr]) && (!exitMask[keyNr]))
      {
        keyPressed1 = keyNr;
      }
      if(deziMask[keyNr])
      {
        if (millis()-deziTimer<=DEZIPRESS)
        {
          if (millis()-deziTimer>=PRELLO)
          {
            keyPressed1=keyNr|0x20;
            deziTimer=millis();
          }
        }
      }
    }
    else
    {
      // No, not newly pressed
      if (longMask[keyNr])
      {
        // Langdruck der Taste abfragen
        if (millis()-timer>=LONGPRESS)
        {
          keyPressed1=keyNr|0x40;
        }
      }
      else
      {
        if (repeatMask[keyNr])
        {
          // Repeatdruck der Taste abfragen
          if (millis() - timer >= REPEATPRESS) {
            timer = millis();
            keyPressed1 = keyNr;
            oldKey[keyNr] = false;
          }
        }
        else
        {
          // Exitdruck der Tasten abfragen
          if (exitMask[keyNr]) {
            if (millis() - timer > EXITPRESS) {
              keyPressed1=keyNr|0x80;
              ready = true;
            }
          }
        }
      }
    }
  }
  else
  {
    // No, pressed before?
    if(oldKey[keyNr])
    {
      if (millis()-timer>PRELLO)
      {
        oldKey[keyNr] = false;
        // Yes, pressed before
        if(deziMask[keyNr])
          deziTimer=millis();
        if(longMask[keyNr])
        {
          if (keyPressed1<0)
          {
            if (millis()-timer<LONGPRESS)
            {
              keyPressed1=keyNr;
            }
            else
            {
              keyPressed1=keyNr|0x40;
            }
          }
          timer = millis();
        }
        else
        {
          if (repeatMask[keyNr])
          {
            oldKey[keyNr] = false;
          }
          else
          {
            if (exitMask[keyNr])
            {
              oldKey[keyNr] = false;
            }
            else
            {
              oldKey[keyNr] = false;
            }
          }
        }
      }
      else
      {
        timer=millis();
      }
    }
  }
  return (keyPressed1);
}

/* int scanKeys()
  Scans all Keys for Key-presses
*/
int scanKeys()
{
  int8_t keyPressed;
  uint8_t i;
  keyPressed = -1;
  for (i = 0; i < 4; i++)
  {
    keyPressed = scanKey(i);
    if(keyPressed >= 0) break;
  }
  return (keyPressed);
}

/* refreshDisplay(int mode)
  Refreshes the display
*/
void refreshDisplay(int mode)
{
  uint8_t i, flag, posy;
  display.clearDisplay();
  flag=false;
  switch(mode)
  {
    case 0: // Display: Controller and Midi-Channel
      flag=false;
      for (i=0;i<MAXCHANNELS;i++)
      {
        if (i!=actPort)
        {
          if ((midiController[actPort]==midiController[i])&&(midiChannel[actPort]==midiChannel[i]))
          {
            flag=true;
          }
        }
#ifdef SER2
        //sprintf(s,"%d",flag); Serial.println(s);
#endif
      }
      if(flag)
      {
        display.setTextColor(SSD1306_BLACK);
        display.fillRect(0, 0, 128, 64, SSD1306_WHITE);
      }
      else
      {
        display.setTextColor(SSD1306_WHITE);
      }
      display.setCursor(0, 0);
      display.println("Controller");
      sprintf(s, "PORT %d", actPort + 1);
      display.println(s);
      sprintf(s, "CONT %3d", midiController[actPort]);
      for (i = 5; i < strlen(s); i++)
        if (s[i] == ' ') s[i] = '0';
      display.println(s);
      sprintf(s, "CHN  %2d", midiChannel[actPort] + 1);
      for (i = 5; i < strlen(s); i++)
        if (s[i] == ' ') s[i] = '0';
      display.println(s);
      if (inputMode == 0) posy = 38;
      else posy = 54;
      if (!flag)
        display.fillCircle(117, posy, 5, SSD1306_WHITE);
      else
        display.fillCircle(117, posy, 5, SSD1306_BLACK);
      display.display();
      refresh = false;
      break;
    case 1: // Display: Min and Max-Values
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("MinMax");
      sprintf(s, "PORT %d", actPort + 1);
      display.println(s);
      sprintf(s, "MIN %3d", midiMin[actPort]);
      for (i = 4; i < strlen(s); i++)
        if (s[i] == ' ') s[i] = '0';
      display.println(s);
      sprintf(s, "MAX %3d", midiMax[actPort]);
      for (i = 4; i < strlen(s); i++)
        if (s[i] == ' ') s[i] = '0';
      display.println(s);
      if (inputMode == 0) posy = 22+16;
      else posy = 38+16;
      if (!flag)
        display.fillCircle(128 - 11, posy, 5, SSD1306_WHITE);
      else
        display.fillCircle(128 - 11, posy, 5, SSD1306_BLACK);
      display.display();
      refresh = false;
      break;
  }
}

/* int checkEEPROM()
  Checks wheather the first bytes (text) are written right in EEPROM
*/
int checkEEPROM()
{
  char name[]="EFFECTBRIDGE 3.0";
  char a;
  uint8_t i,flag;
  // Checks, if Data inside EEPROM are stored by the program
  int eeAddress;
  eeAddress=0;
  flag=true;
  for(i=0;i<strlen(name);i++)
  {
    EEPROM.get(eeAddress,a);
    if(a!=name[i]) flag=false;
    eeAddress++;
  }
#ifdef SER2
  sprintf(s,"READ EEPROM FLAG %d",flag);
  //Serial.println(s);
#endif
  return(flag);
}

/* int writeEEPROM()
  Writes actual state to EEPROM
*/
int writeEEPROM()
{
  char name[]="EFFECTBRIDGE 3.0";
  uint8_t i;
  int eeAddress;
  eeAddress=0;
  for(i=0;i<strlen(name);i++)
  {
    EEPROM.put(eeAddress,name[i]);
    eeAddress++;
  }
  for(i=0;i<MAXCHANNELS;i++)
  {
    EEPROM.put(eeAddress,midiController[i]);
    eeAddress++;
    EEPROM.put(eeAddress,midiChannel[i]);
    eeAddress++;
    EEPROM.put(eeAddress,midiMin[i]);
    eeAddress++;
    EEPROM.put(eeAddress,midiMax[i]);
    eeAddress++;
  }
#ifdef SER2
  sprintf(s,"WRITE EEPROM %d bytes",eeAddress);
  //Serial.println(s);
#endif
  return(eeAddress);
}

/* int readEEPROM()
  reads saved settings to actual state
*/
int readEEPROM()
{
  char name[]="EFFECTBRIDGE 3.0";
  char a;
  uint8_t i,flag;
  // Checks, if Data inside EEPROM are stored by the program
  int eeAddress;
  eeAddress=0;
  flag=true;
  for(i=0;i<strlen(name);i++)
  {
    EEPROM.get(eeAddress,a);
    if(a!=name[i]) flag=false;
    eeAddress++;
  }
  if (flag)
  {
    for(i=0;i<MAXCHANNELS;i++)
    {
      EEPROM.get(eeAddress,midiController[i]);
      eeAddress++;
      EEPROM.get(eeAddress,midiChannel[i]);
      eeAddress++;
      EEPROM.get(eeAddress,midiMin[i]);
      eeAddress++;
      EEPROM.get(eeAddress,midiMax[i]);
      eeAddress++;
    }
  }
#ifdef SER2
  sprintf(s,"EEPROM %d bytes",eeAddress);
  //Serial.println(s);
#endif
  return(eeAddress);
}

/* void EditMidi(int mode)
  Edits the actual Midi-Settings
*/
void EditMidi(int mode)
{
  int keyPressed;
  display.setTextSize(2);
  do
  {
    if (refresh)
    {
      refreshDisplay(mode);
    }
    keyPressed = scanKeys();
    switch (keyPressed)
    {
      case 0:
        if (actPort > 0) actPort--;
        else actPort = MAXCHANNELS-1;
        refresh = true;
        break;
      case 1:
        switch(mode)
        {
          case 0:
            switch (inputMode)
            {
              case 0:
                if (midiController[actPort] < 150) midiController[actPort]++;
                else midiController[actPort] = 0;
                refresh = true;
                break;
              case 1:
                if (midiChannel[actPort] < 15) midiChannel[actPort]++;
                else midiChannel[actPort] = 0;
                refresh = true;
                break;
            }
            break;
          case 1:
            switch (inputMode)
            {
              case 0:
                if (midiMin[actPort] < 100) midiMin[actPort]++;
                else midiMin[actPort] = 0;
                refresh = true;
                break;
              case 1:
                if (midiMax[actPort] < 100) midiMax[actPort]++;
                else midiMax[actPort] = 0;
                refresh = true;
                break;
            }
            break;
        }
        break;
      case 2:
        switch(mode)
        {
          case 0:
            switch (inputMode)
            {
              case 0:
                if (midiController[actPort] > 0) midiController[actPort]--;
                else midiController[actPort] = 150;
                refresh = true;
                break;
              case 1:
                if (midiChannel[actPort] > 0) midiChannel[actPort]--;
                else midiChannel[actPort] = 15;
                refresh = true;
                break;
            }
            break;
          case 1:
            switch (inputMode)
            {
              case 0:
                if (midiMin[actPort] > 0) midiMin[actPort]--;
                else midiMin[actPort] = 100;
                refresh = true;
                break;
              case 1:
                if (midiMax[actPort] > 0) midiMax[actPort]--;
                else midiMax[actPort] = 100;
                refresh = true;
                break;
            }
            break;
        }
        break;
      case 3:
        if (actPort < MAXCHANNELS-1) actPort++;
        else actPort = 0;
        refresh = true;
        break;
      case 0x41:
        inputMode = 0; refresh=true;
        break;
      case 0x42:
        inputMode = 1; refresh=true;
        break;
      case 0x40:
        ready=true;
        break;
      case 0x43:
        ready=true;
        break;
      case 0x21:
        deziTimer=millis();
        switch(mode)
        {
          case 0:
            switch (inputMode)
            {
              case 0:
                if (midiController[actPort] < 150) midiController[actPort]+=8;
                else midiController[actPort] = 0;
                refresh = true;
                break;
              case 1:
                if (midiChannel[actPort] < 15) midiChannel[actPort]++;
                else midiChannel[actPort] = 0;
                refresh = true;
                break;
            }
            break;
          case 1:
            switch (inputMode)
            {
              case 0:
                if (midiMin[actPort] < 90) midiMin[actPort]+=8;
                else midiMin[actPort] = 0;
                refresh = true;
                break;
              case 1:
                if (midiMax[actPort] < 90) midiMax[actPort]+=8;
                else midiMax[actPort] = 0;
                refresh = true;
                break;
            }
            break;
        }
        break;
      case 0x22:
        deziTimer=millis();
        switch(mode)
        {
          case 0:
            switch (inputMode)
            {
              case 0:
                if (midiController[actPort] >= 10) midiController[actPort]-=8;
                else midiController[actPort] = 150;
                refresh = true;
                break;
              case 1:
                if (midiChannel[actPort] > 0) midiChannel[actPort]--;
                else midiChannel[actPort] = 15;
                refresh = true;
                break;
            }
            break;
          case 1:
            switch (inputMode)
            {
              case 0:
                if (midiMin[actPort] >= 10) midiMin[actPort]-=8;
                else midiMin[actPort] = 100;
                refresh = true;
                break;
              case 1:
                if (midiMax[actPort] >=10) midiMax[actPort]-=8;
                else midiMax[actPort] = 100;
                refresh = true;
                break;
            }
            break;
        }
        break;
    }
  } while (!ready);
}

/* int ScanEditReset()
  scans Keys while performing for long keypresses
*/
int ScanEditReset()
{
  int keyPressed;
  int action;
  action=0;
  ready=false;
    keyPressed = scanKeys();
    switch (keyPressed)
    {
      case 0x40:
        action=1;
        ready=true;
        break;
      case 0x43:
        action=2;
        ready=true;
        break;
    }
//  } while (!ready);
  return(action);
}

/* void setup()
  Main-program
*/
void setup()
{
  float dummy;
  //uint16_t servoPos[3];
#ifndef NEWCALC  
  float fact=(2100-530)/128;
  float factv=4095/128;
#endif
  midiEventPacket_t rx;
  int keyPressed;
  uint8_t displayo;

#ifdef SSER
  Serial.begin(115200);
#endif  

#ifdef FRQ400KHZ
  Wire.setClock(400000);
#endif

  midiController[0] = 7;
  midiChannel[0] = 0;
  midiController[1] = 7;
  midiChannel[1] = 1;
  midiController[2] = 7;
  midiChannel[2] = 2;
  midiController[3] = 7;
  midiChannel[3] = 3;
  midiController[4] = 7;
  midiChannel[4] = 4;
  midiController[5] = 7;
  midiChannel[5] = 5;
  midiController[6] = 7;
  midiChannel[6] = 6;
  midiController[7] = 7;
  midiChannel[7] = 7;
  midiMin[0]=midiMin[1]=midiMin[2]=midiMin[3]=0;
  midiMax[0]=midiMax[1]=midiMax[2]=midiMax[3]=100;
  midiMin[4]=midiMin[5]=midiMin[6]=midiMin[7]=0;
  midiMax[4]=midiMax[5]=midiMax[6]=midiMax[7]=100;
  myservo1.attach(PSERVO1);
  myservo2.attach(PSERVO2);
  myservo3.attach(PSERVO3);
  myservo4.attach(PSERVO4);

  inputMode = 0;
  oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
  actPort = 0;
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
#ifdef SER1
      Serial.println(F("SSD1306 allocation failed"));
#endif
    for (;;)
      ;  // Don't proceed, loop forever
  }
  if(!mcp.begin())
  {
    #ifdef SER1
      Serial.println(F("MCP4728 allocation failed"));
    #endif
    for (;;)
      ;  // Don't proceed, loop forever

  }
  Serial.println(F("Serial Loeppt!"));
  mcp.setChannelValue(MCP4728_CHANNEL_A, 0,MCP4728_VREF_INTERNAL,MCP4728_GAIN_2X);
  mcp.setChannelValue(MCP4728_CHANNEL_B, 0,MCP4728_VREF_INTERNAL,MCP4728_GAIN_2X);
  mcp.setChannelValue(MCP4728_CHANNEL_C, 0,MCP4728_VREF_INTERNAL,MCP4728_GAIN_2X);
  mcp.setChannelValue(MCP4728_CHANNEL_D, 0,MCP4728_VREF_INTERNAL,MCP4728_GAIN_2X);

  pinMode(KEYLEFT, INPUT_PULLUP);
  pinMode(KEYUP, INPUT_PULLUP);
  pinMode(KEYDOWN, INPUT_PULLUP);
  pinMode(KEYRIGHT, INPUT_PULLUP);
  display.display();
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text

  display.cp437(true);  // Use full 256 char 'Code Page 437' font

  ready = false;
  refresh = true;
  if(!checkEEPROM())
  //if(1==1)
  {
    EditMidi(0);
    oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
    EditMidi(1);
/*    for(i=0;i<4;i++)
    {
      {
        refresh=midiMin[i]; midiMin[i]=midiMax[i]; midiMax[i]=refresh;
        refresh=true;
      }
    }
*/
    if(writeEEPROM()==LENVERSION+LENDATA)
    {
      display.clearDisplay();
       display.drawBitmap(0, 0, myBitmap, 128, 20,SSD1306_WHITE);
      display.setTextSize(2);
      display.setCursor(0,FZEIL);
      display.println("WR OK");
      display.display();
      delay(1000);
    }
    else
    {
      display.clearDisplay();
      display.drawBitmap(0, 0, myBitmap, 128, 20,SSD1306_WHITE);
      display.setTextSize(2);
      display.setCursor(0,FZEIL);
      display.println("WR FALSE");
      display.display();
      delay(1000);
    }
  }
  else
  {
    if(readEEPROM()==LENVERSION+LENDATA)
    {
      display.clearDisplay();
      display.drawBitmap(0, 0, myBitmap, 128, 20,SSD1306_WHITE);
      display.setTextSize(2);
      display.setCursor(0,FZEIL);
      display.println("WR OK");
      display.display();
      delay(1000);
    }
    else
    {
      display.clearDisplay();
      display.drawBitmap(0, 0, myBitmap, 128, 20,SSD1306_WHITE);
      display.setTextSize(2);
      display.setCursor(0,FZEIL);
      display.println("WR FALSE");
      display.display();
      delay(1000);
    }
  }
  displayo=true;
  oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
  ready = false;
  refresh = true;
  inputMode = 0;
  actPort = 0;
  for(i=0;i<4;i++)
  {
    oldservoPos[i]=servoPos[i];
    oldvoltage[i]=voltage[i];
  }
  milliSave2=milliSave=millis();
  
  
  do
  {
    if(displayo)
    {
      display.clearDisplay();
      display.drawBitmap(0, 0, myBitmap, 128, 20,SSD1306_WHITE);
      display.setTextSize(1);
      display.setCursor(0,FZEIL);
      display.println("Version 0.2");
      display.println("by Hartmut Wagener");
      display.display();
      delay(2000);
      display.fillRect(0,20,128,108, SSD1306_BLACK);
      display.setTextSize(1);
      display.setCursor(0,FZEIL);
      display.println("L-LONGPRESS: EDIT");
      display.println("R-LONGPRESS: Default");
      display.display();
      displayo=false;
    }
    ready=false;
    keyPressed=ScanEditReset();
    ready=false;
    if(keyPressed>0)
    {
      ready=false;
      do
      {
        if((digitalRead(KEYLEFT))&&(digitalRead(KEYRIGHT))) ready=true;
      } while(!ready);
    }
    //oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
    switch(keyPressed)
    {
      case 1:
        ready = false;
        refresh = true;
        inputMode = 0;
        oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
        actPort = 0;
        EditMidi(0);
        ready=false;
        do
        {
          if((digitalRead(KEYLEFT))&&(digitalRead(KEYRIGHT))) ready=true;
        } while(!ready);
        keyPressed=0;
        oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
        ready=false;
        refresh=true;
        inputMode = 0;
        EditMidi(1);
        ready=false;
        do
        {
          if((digitalRead(KEYLEFT))&&(digitalRead(KEYRIGHT))) ready=true;
        } while(!ready);
        keyPressed=0;
        oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
        ready=false;
        refresh=true;

        if(writeEEPROM()==LENVERSION+LENDATA)
        {
          display.clearDisplay();
          display.drawBitmap(0, 0, myBitmap, 128, 20,SSD1306_WHITE);
          display.setTextSize(2);
          display.setCursor(0,FZEIL);
          display.println("WR OK");
          display.display();
          delay(1000);
        }
        else
        {
          display.clearDisplay();
          display.drawBitmap(0, 0, myBitmap, 128, 20,SSD1306_WHITE);
          display.setTextSize(2);
          display.setCursor(0,FZEIL);
          display.println("WR ERR");
          display.display();
          delay(1000);
        }
        displayo=true;
        break;
      case 2:
        midiController[0] = 70;
        midiChannel[0] = 0;
        midiController[1] = 71;
        midiChannel[1] = 0;
        midiController[2] = 72;
        midiChannel[2] = 0;
        midiController[3] = 73;
        midiChannel[3] = 0;
        ready = false;
        refresh = true;
        inputMode = 0;
        oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
        actPort = 0;
        EditMidi(0);
        if(writeEEPROM()==LENVERSION+LENDATA)
        {
          display.clearDisplay();
          display.drawBitmap(0, 0, myBitmap, 128, 20,SSD1306_WHITE);
          display.setTextSize(2);
          display.setCursor(0,FZEIL);
          display.println("WR OK");
          display.display();
          delay(1000);
        }
        else
        {
          display.clearDisplay();
          display.drawBitmap(0, 0, myBitmap, 128, 20,SSD1306_WHITE);
          display.setTextSize(2);
          display.setCursor(0,FZEIL);
          display.println("WR ERR");
          display.display();
          delay(1000);
        }
        displayo=true;
        break;
    }
    if (keyPressed>0)
    {
      ready=false;
      do
      {
        if((digitalRead(KEYLEFT))&&(digitalRead(KEYRIGHT))) ready=true;
      } while(!ready);
      oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
    }
    keyPressed=0;
    rx=MidiUSB.read();
#ifdef SER2
    //Serial.println(".");
#endif
    if (rx.header!=0)
    {
#ifdef SER2
      sprintf(s,"%d %d %d %d",rx.header,rx.byte1,rx.byte2,rx.byte3);
      // Serial.println(s);
#endif
      for(i=0;i<MAXCHANNELS;i++)
      {
        if (((rx.byte1&0xF0)==0xB0)&&((rx.byte1&0x0F)==midiChannel[i]))
        {
          if(rx.byte2==midiController[i])
          {
            if(i<MAXCHANNELS/2)
            {
#ifdef NEWCALC
              dummy=(float)rx.byte3;
              dummy=dummy/127.0;
              servoPos[i]=CalcMS(dummy,midiMin[i],midiMax[i]);
#else
              servoPos[i]=530+(uint16_t)((float)rx.byte3*fact);
#endif
            }
            else
            {
#ifdef NEWCALC
              dummy=(float)rx.byte3;
              dummy=dummy/127.0;
//              voltage[i-MAXCHANNELS/2]=(uint16_t)((float)rx.byte3*factv);
              voltage[i-MAXCHANNELS/2]=CalcVOLT(dummy,midiMin[i],midiMax[i]);
//                voltage[i-MAXCHANNELS/2]=4095;
#else
              voltage[i-MAXCHANNELS/2]=(uint16_t)((float)rx.byte3*factv);
#endif
            }
          }
        }
      }
      milliflag=milliSave2+MILLIPLUS>millis();
      if(milliflag) milliSave2=millis();
      for(i=0;i<4;i++)
      {
        //sprintf(s,"%d",voltage[i]);
        //Serial.println(s);
        switch(i)
        {
          case 0:
            if(oldservoPos[i]!=servoPos[i])
            {
              myservo1.writeMicroseconds(servoPos[i]);
            }
            if((abs(oldvoltage[i]-voltage[i])>VOLTDIF)||(milliflag))
            {
              mcp.setChannelValue(MCP4728_CHANNEL_A,voltage[i]);
            }
            break;            
          case 1:
            if(oldservoPos[i]!=servoPos[i])
            {
              myservo2.writeMicroseconds(servoPos[i]);
            }
            if((abs(oldvoltage[i]-voltage[i])>VOLTDIF)||(milliflag))
            {
              mcp.setChannelValue(MCP4728_CHANNEL_B,voltage[i]);
            }
            break;            
          case 2:
            if(oldservoPos[i]!=servoPos[i])
            {
              myservo3.writeMicroseconds(servoPos[i]);
            }
            if((abs(oldvoltage[i]-voltage[i])>VOLTDIF)||(milliflag))
            {
              mcp.setChannelValue(MCP4728_CHANNEL_C,voltage[i]);
            }
            break;            
          case 3:
            if(oldservoPos[i]!=servoPos[i])
            {
              myservo4.writeMicroseconds(servoPos[i]);
            }
            if((abs(oldvoltage[i]-voltage[i])>VOLTDIF)||(milliflag))
            {
              mcp.setChannelValue(MCP4728_CHANNEL_D,voltage[i]);
            }
            break;            
        }        
      }
    }
    if(millis()>milliSave+500)
    {
      milliSave=millis();
      display.fillRect(X1ACT-RACT,YACT-RACT,9*XACT+2*RACT,2*RACT,SSD1306_BLACK);
      for(i=0;i<MAXCHANNELS;i++)
      {
        if(i<MAXCHANNELS/2)
        {
          if(servoPos[i]!=oldservoPos[i])
          {
            oldservoPos[i]=servoPos[i];
            display.fillCircle(X1ACT+i*XACT,YACT,RACT,SSD1306_WHITE);
          }
          else
          {
            display.drawCircle(X1ACT+i*XACT,YACT,RACT,SSD1306_WHITE);
          }
        }
        else
        {
          if(voltage[i-MAXCHANNELS/2]!=oldvoltage[i-MAXCHANNELS/2])
          {
            oldvoltage[i-MAXCHANNELS/2]=voltage[i-MAXCHANNELS/2];
            display.fillCircle(X1ACT+XACT/2+i*XACT,YACT,RACT,SSD1306_WHITE);
          }
          else
          {
            display.drawCircle(X1ACT+XACT/2+i*XACT,YACT,RACT,SSD1306_WHITE);
          }
        }
      }
      display.display();
    }
  } while(1==1);
}

/* void loop()
  Loop-part of Arduino-Sketch
*/
void loop()
{
}
