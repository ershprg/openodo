//Display library
#include <LEDDisplayDriver.h>
#ifndef _HT1621_6D_
#error "_HT1621_6D_ must be defined in LEDDisplayDriver.h for this sketch to work"
#endif

#include <GPSport.h>

//----HT1621 var
#define LCD_DATA_PIN 7
#define LCD_WR_PIN 12
#define LCD_CS_PIN 13
LEDDisplayDriver display(LCD_DATA_PIN, LCD_WR_PIN,LCD_CS_PIN,false,6);  // 6 digit but 5 digit display

//==========================================================
// Define the pins used for the display connection
//Pin for internal LED
const int ledPin = 13;
const int bt1Pin = 2;//D2
const int bt2Pin = 3;//D3
//Minimal Speed to detect
const int c_vMin = 3;
//Odometer limit in KM
const int c_odoLimit = 10000; //100.00
//Odometer Grade 10 or 100 meters
const int c_odoGrade = 10;
//We read the Odometer at 1Hz
const int c_odoHz = 1;
//==========================================================

//Pin Data
volatile int pin1 = 0;
volatile int pin2 = 0;
volatile long lastDebounceTime = 0;
long debounceDelay = 1500;

//GPS Data after reading
uint32_t speed = 0;
uint16_t hdg = 0;
bool hdg_valid = false;
uint8_t sats = 0;

//ICO internal storage
uint32_t dist_l = 0; //In Meters*3600
uint8_t dist_h = 0; //In KMs

bool ready_to_go = false;
int display_mode = 0; //What is displayed: Speed, Hdg, ODO...

//Displayed values
int i_speed = -1; //-1 -> xxxxx
int i_hdg = -1; //0-359
int i_sats = -1; //-1 -> ~20
uint16_t i_dist_l = 0;//Odometer distance, low, i.e. 1645 = 16km 450m
uint8_t  i_dist_h = 0;//Odometer distance, high (0-99km) //Not needed!
char display_buf[8];

byte greeting[] = {digitG, digitP, digitS, digitO, digitD};


static void initDisplay()
{
  display.clear();
  display.showTest();
  delay(1500);
  display.clear();
  display.showDigits(greeting, 0, 5);
  delay(1500);
 
}

//-----------
// Buttons processing
//-----------
void key1Interrupt() {
//  if ( (millis() - lastDebounceTime) > debounceDelay) {  
    pin1 += 1;
//    lastDebounceTime = millis();
//  }    
}

void key2Interrupt() {
//  if ( (millis() - lastDebounceTime) > debounceDelay) {
    pin2 += 1;
//    lastDebounceTime = millis();
//  }
}


static void processButtons()
{

/*    if (pin1 > 0) {
      pin1 = 0;
      display_mode++;
    }
    if (pin2 > 0) {
      pin2 = 0;
      display_mode--;
    }*/
    if (display_mode < 0)
      display_mode = 0;

    if (display_mode > 5)
      display_mode = 5;
}


//----------------------------------------------------------------
//  This function gets called about once per second, during the GPS
//  quiet time.  It's the best place to do anything that might take
//  a while: print a bunch of things, write to SD, send an SMS, etc.
//
//  By doing the "hard" work during the quiet time, the CPU can get back to
//  reading the GPS chars as they come in, so that no chars are lost.

static void displayData()
{
  byte indic = 0;
  sprintf(display_buf," %2d", display_mode);
  display.showText(display_buf);
  display.update();

  int val1 = digitalRead(A0);
  int val2 = digitalRead(A1);

  DEBUG_PORT.print(F("P1 "));
  DEBUG_PORT.print(val1);
  DEBUG_PORT.print(F(" P2 "));
  DEBUG_PORT.print(val2);
  DEBUG_PORT.print(F(" LAST "));
  DEBUG_PORT.print(lastDebounceTime);
  DEBUG_PORT.print(F(" Mode "));
  DEBUG_PORT.print(display_mode);
  DEBUG_PORT.println();

}
//------------------------------------
//  This is the main GPS parsing loop.

static void GPSloop()
{
  processButtons();
  displayData();

} // GPSloop

//--------------------------


void setup()
{

  initDisplay();
  
  DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT)
    ;

  DEBUG_PORT.flush();


  //Init Button Pins
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
//  pinMode(23, INPUT); 
//  pinMode(24, INPUT); 

//  attachInterrupt(digitalPinToInterrupt(bt1Pin), key1Interrupt, RISING);
//  attachInterrupt(digitalPinToInterrupt(bt2Pin), key2Interrupt, RISING);
  
}

//--------------------------

void loop()
{
  GPSloop();
}
