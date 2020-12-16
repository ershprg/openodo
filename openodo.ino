//Power
#include <avr/wdt.h>
#include <avr/sleep.h>

//PinChangeInterrupt
#include <PinChangeInterrupt.h>

//GPIO Library
#include <FastGPIO.h>

//Display library
#include <LEDDisplayDriver.h>
#ifndef _HT1621_6D_
#error "_HT1621_6D_ must be defined in LEDDisplayDriver.h for this sketch to work"
#endif

//GPS Library
#include <NMEAGPS.h>

//======================================================================
//  Program: NMEA.ino
//
//  Description:  This program uses the fix-oriented methods available() and
//    read() to handle complete fix structures.
//
//    When the last character of the LAST_SENTENCE_IN_INTERVAL (see NMEAGPS_cfg.h)
//    is decoded, a completed fix structure becomes available and is returned
//    from read().  The new fix is saved the 'fix' structure, and can be used
//    anywhere, at any time.
//
//    If no messages are enabled in NMEAGPS_cfg.h, or
//    no 'gps_fix' members are enabled in GPSfix_cfg.h, no information will be
//    parsed, copied or printed.
//
//  Prerequisites:
//     1) Your GPS device has been correctly powered.
//          Be careful when connecting 3.3V devices.
//     2) Your GPS device is correctly connected to an Arduino serial port.
//          See GPSport.h for the default connections.
//     3) You know the default baud rate of your GPS device.
//          If 9600 does not work, use NMEAdiagnostic.ino to
//          scan for the correct baud rate.
//     4) LAST_SENTENCE_IN_INTERVAL is defined to be the sentence that is
//          sent *last* in each update interval (usually once per second).
//          The default is NMEAGPS::NMEA_RMC (see NMEAGPS_cfg.h).  Other
//          programs may need to use the sentence identified by NMEAorder.ino.
//     5) NMEAGPS_RECOGNIZE_ALL is defined in NMEAGPS_cfg.h
//
//  'Serial' is for debug output to the Serial Monitor window.
//
//  License:
//    Copyright (C) 2014-2017, SlashDevin
//
//    This file is part of NeoGPS
//
//    NeoGPS is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    NeoGPS is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with NeoGPS.  If not, see <http://www.gnu.org/licenses/>.
//
//======================================================================

//-------------------------------------------------------------------------
//  The GPSport.h include file tries to choose a default serial port
//  for the GPS device.  If you know which serial port you want to use,
//  edit the GPSport.h file.

#include <GPSport.h>

//------------------------------------------------------------
// For the NeoGPS example programs, "Streamers" is common set
//   of printing and formatting routines for GPS data, in a
//   Comma-Separated Values text format (aka CSV).  The CSV
//   data will be printed to the "debug output device".
// If you don't need these formatters, simply delete this section.


//TODO: Remove
#include <Streamers.h>

//------------------------------------------------------------
// This object parses received characters
//   into the gps.fix() data structure

static NMEAGPS  gps;

//------------------------------------------------------------
//  Define a set of GPS fix information.  It will
//  hold on to the various pieces as they are received from
//  an RMC sentence.  It can be used anywhere in your sketch.

static gps_fix  fix;

//----HT1621 var
#define LCD_DATA_PIN 7
#define LCD_WR_PIN 12
#define LCD_CS_PIN 13
LEDDisplayDriver display(LCD_DATA_PIN, LCD_WR_PIN,LCD_CS_PIN,false,6);  // 6 digit but 5 digit display

// GPS to 5Hz
const unsigned char ubxRate1Hz[] PROGMEM = 
  { 0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00 };
const unsigned char ubxRate5Hz[] PROGMEM =
  { 0x06,0x08,0x06,0x00,200,0x00,0x01,0x00,0x01,0x00 };
const unsigned char ubxRate10Hz[] PROGMEM =
  { 0x06,0x08,0x06,0x00,100,0x00,0x01,0x00,0x01,0x00 };

//Power State (Actual Condition)
#define BAT_ON 1 //On battery
#define IGN_ON 0 //On 12v

// Power Mode
#define ON_PWR 0 //Is On
#define OFF_PWR 1 //Is Off
#define BATT_PWR 2 //Is On on Battery (1 -> 2)
#define SW_ON_PWR 3 //Turning On (1,2) -> 2 -> 0
#define SH_DOWN_PWR 4 //Turning Off (0,2) -> 3 -> 1

//==========================================================
// Define the pins used for the display connection
//Pin for internal LED
const int ledPin = 13;
const int bt1Pin = A0;
const int bt2Pin = A1;
const int pwrPin = 2;

//Minimal Speed to detect
const int16_t c_vMin = 5L*1000*5; // = 5x GPS Reported Speed (Meters per hour)
//Odometer limit 
const int16_t c_odoLimit = 1000*10; //100.00
//Odometer Grade 10 or 100 meters
const int8_t c_odoGrade = 10;
//We read the Odometer at 1Hz
const int8_t c_odoHz = 5;
//==========================================================
//Convert meters per hour every 1 second to km/meters with required precision
const uint32_t c_dist_divider = (3600UL * c_odoGrade * c_odoHz);

//For Odometer rolling (buton pressed)
const uint32_t c_dist_divider_slower = (36UL * c_odoGrade * c_odoHz);

//Maximum reachable odometer is 100km
const uint32_t c_odoMax = (3600UL*1000*100*c_odoHz);

//Pin Data
volatile bool pin1_short = false;
volatile uint8_t pin1_long = 0; //0, 1 is waiting for long, 2 is Pressed
volatile bool pin2_short = false;
volatile uint8_t pin2_long = 0; //0, 1 is waiting for long, 2 is Pressed
volatile long lastDebounceTime_pin1 = 0;
volatile long lastDebounceTime_pin2 = 0;
volatile long lastDebounceTime_pwr = 0;

long debounceDelay = 100;
long debounceDelay_up = 50;
long longPressDelay = 750;
long debounceDelay_pwr = 1000;

//GPS Data after reading
uint32_t speed = 0;
uint16_t hdg = 0;
bool hdg_valid = false;
uint8_t sats = 0;

uint32_t last5speeds = 0; //Sum of last 5 speeds
uint32_t last25speeds = 0; //Sum of last 25 speeds
bool stopped = true;
//ICO internal storage
int32_t dist_l = 0; //In Meters*3600


bool ready_to_go = false;
int display_mode = 0; //What is displayed: Speed, Hdg, ODO...
volatile int power_state = IGN_ON; //Power STATE as detected by the Interrupt; 1 = battery, 0 = online
volatile int power_mode = ON_PWR; //Power MODE as the device needs 
int menu_mode = 0;

//Displayed values
int i_speed = -1; //-1 -> xxxxx
int i_hdg = -1; //0-359
int i_sats = -1; //-1 -> ~20
int8_t i_dist_l = 0;//Odometer distance, low, i.e. 1645 = 16km 450m
int8_t i_dist_h = 0;//Odometer distance, high (0-99km) //Not needed!
char display_buf[8];

byte greeting[] = {digitG, digitP, digitS, digitO, digitD};
byte sw_version[] = {digitR, digitE, digitL, digit0, digit1};

void sendUBX( const unsigned char *progmemBytes, size_t len )
{
  gpsPort.write( 0xB5 ); // SYNC1
  gpsPort.write( 0x62 ); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte( progmemBytes++ );
    a += c;
    b += a;
    gpsPort.write( c );
  }

  gpsPort.write( a ); // CHECKSUM A
  gpsPort.write( b ); // CHECKSUM B

} // sendUBX


//TODO: Rewrite properly for Arduino
/*uint32_t divu10_32(uint32_t n) {
    uint32_t q, r;
    q = (n >> 1) + (n >> 2);
    q = q + (q >> 4);
    q = q + (q >> 8);
    q = q + (q >> 16);
    q = q >> 3;
    r = n - (((q << 2) + q) << 1);
    return q + (r > 9);
}

*/

static void initDisplay()
{
  display.showTest();
  display.update();
  delay(500);
  display.clear();
}

static void greetDisplay()
{
  display.showDigits(greeting, 0, 5);
  display.update();
  delay(500);
  display.showDigits(sw_version, 0, 5);
  display.update();
  delay(500);
  display.clear();
}

//------------------
// Convert GPS data to be able to display the values
//------------------
static void processData()
{
  //TODO: avoid division
  i_sats = sats; //Direct conversion (can convert to a signal level, i.e. 0 Sats, <3 Sats, >5 Sats)
  
  if (hdg_valid) //If we are not moving/do not know, assume it does not change
    i_hdg = hdg / 100; //NanoGPS gives in 100s fractions 
    
  i_speed = speed / 1000; //NanoGPS gives in meters per hour 
  last5speeds = (4UL * last5speeds / 5UL) + speed;
  last25speeds = (24UL * last25speeds / 25UL) + speed;
}

//-----------
// Odometer increase and carryover
//-----------
static void processOdometer()
{

  if (last5speeds > c_vMin) { //1 last second of movement
    stopped = false;
    //dist_l += last5speeds * 4;//1 more below //WONT WORK
  } else if (last25speeds < c_vMin * 5UL) //5 last seconds in PAUSE
      stopped = true;

  if (stopped == false) {
    dist_l += speed; //@1 Hz, would be 5x more if 5Hz
    if (dist_l > c_odoMax) {//We sum Meters Per Hour up to 100kms. 100km -> 100*1000m = 100*1000*3600m/3600sec
      dist_l -= c_odoMax ;
    }
  }


/*    uint32_t li_dist_l = dist_l / c_dist_divider; //Divide by 3600 (hour -> second) and by X (dist in 10s or 100s of meters)

    //i_dist_l and i_dist_h here would be 16 and 68 here -> 16km 680m
    i_dist_h = li_dist_l / 100UL;
    i_dist_l = li_dist_l % 100UL;
    */
}

static void updateOdometer()
{
    uint32_t li_dist_l = dist_l / c_dist_divider; //Divide by 3600 (hour -> second) and by X (dist in 10s or 100s of meters)

    //i_dist_l and i_dist_h here would be 16 and 68 here -> 16km 680m
    i_dist_h = li_dist_l / 100UL;
    i_dist_l = li_dist_l % 100UL;
  
}
void pwrInterrupt() {
 bool inputHigh = FastGPIO::Pin<pwrPin>::isInputHigh();
  if (inputHigh  == false) {
    if ( (millis() - lastDebounceTime_pwr) > debounceDelay_pwr) {
      lastDebounceTime_pwr = millis();
      power_state = BAT_ON; //Fall to Battery
     //PWR OUT
    }
  } else if(inputHigh  == true) {
    if ( (millis() - lastDebounceTime_pwr) > debounceDelay_pwr) {
      lastDebounceTime_pwr = millis();
      //PWR IN
        power_state = IGN_ON; //Just switching ON
    }
  } 
}

//-----------
// Buttons processing SHOULD be done with an interrupt - however the debounce with ICO controller is terrible: button1 is 4.75v and button2 is 0.85v that triggers them BOTH. Working with interrupt on Analog ports saves the day.
//-----------

void key1Interrupt() {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(bt1Pin));
  if(trigger == FALLING){
    if ( (millis() - lastDebounceTime_pin1) > debounceDelay) {
      lastDebounceTime_pin1 = millis();
      if (pin1_long == 0)
        pin1_long = 1; //Holding
    }
  } else if(trigger == RISING) {
    if ( (millis() - lastDebounceTime_pin1) > debounceDelay_up) {
      if (pin1_long == 2)      
        pin1_long = 3; //Released after long hold
      else {
        pin1_short = true;
        pin1_long = 0;
      }
    }
  } 
}

void key2Interrupt() {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(bt2Pin));
  if (trigger == FALLING) {
    if ( (millis() - lastDebounceTime_pin2) > debounceDelay) {
      lastDebounceTime_pin2 = millis();
      if (pin2_long == 0)      
        pin2_long = 1; //Holding
    }
  } else if(trigger == RISING) {
    if ( (millis() - lastDebounceTime_pin2) > debounceDelay_up) {
      if (pin2_long == 2)
        pin2_long = 3; //Released after hold
      else {
        pin2_short = true;
        pin2_long = 0;
      }
    }
  }  
}

//Interrupt for GPS
static void GPSisr( uint8_t c )
{
  gps.handle( c );

} // GPSisr

static void checkLongPress()
{
  if (pin1_long == 1) {
    if ( (millis() - lastDebounceTime_pin1) > longPressDelay)    
      pin1_long = 2;
  }

  if (pin2_long == 1) {
    if ( (millis() - lastDebounceTime_pin2) > longPressDelay)    
      pin2_long = 2;
  }
}

static void processButtons()
{

    //BOTH SHORT
    if ((pin1_short == true)&&(pin2_short == true)) {
      display_mode++;
      if (display_mode > 3)
        display_mode = 0;
      pin1_short = false;
      pin2_short = false;
      return;
    }

    //SHORT PRESS
    if (pin1_short == true) {
      pin1_short = false;
      dist_l += c_dist_divider ;
      if (dist_l > c_odoMax)
        dist_l -= c_odoMax;
    }
    if (pin2_short == true) {
      pin2_short = false;
      dist_l -= c_dist_divider ;
      if (dist_l < 0)
        dist_l = 0;
    }

}
/*
//This moved here, so runs at 1-5Hz
static void processLongButtonsHertz() {

    //LONG PRESS BOTH - WORKS ONLY ON RELEASE!
    if ((pin1_long == 2)&&(pin2_long == 2)) {
      if (menu_mode == 0)
        menu_mode = 1;
      return;
    }

    //LONG PRESS BOTH - WORKS ONLY ON RELEASE!
    if ((pin1_long == 3)&&(pin2_long == 3)) {
      pin1_long = 0;
      pin2_long = 0;
      if (menu_mode == 1)
        menu_mode = 2;
      return;
    }
  
    //LONG PRESS
    if (pin1_long == 2) {
      dist_l += 3*c_dist_divider ;
      if (dist_l > c_odoMax) {
        dist_l -= c_odoMax;
        pin1_long = 0;
      }
    }

    if (pin1_long == 3)
      pin1_long = 0;
      
    if (pin2_long == 2) {
      dist_l -= 3*c_dist_divider ;
      //dist_l -= c_dist_divider ;
      if (dist_l < 0) {
        dist_l = 0;
        pin2_long = 0;
      }
    }

    if (pin2_long == 3)
      pin2_long = 0;
}*/

//This does NOT run at 1-5Hz
static void processLongButtons() {

    //LONG PRESS BOTH - WORKS ONLY ON RELEASE!
    if ((pin1_long == 2)&&(pin2_long == 2)) {
      if (menu_mode == 0)
        menu_mode = 1;
      return;
    }

    //LONG PRESS BOTH - WORKS ONLY ON RELEASE!
    if ((pin1_long == 3)&&(pin2_long == 3)) {
      pin1_long = 0;
      pin2_long = 0;
      if (menu_mode == 1)
        menu_mode = 2;
      return;
    }
  
    //LONG PRESS
    if (pin1_long == 2) {
      dist_l += c_dist_divider_slower ;
      if (dist_l > c_odoMax) {
        dist_l -= c_odoMax;
        pin1_long = 0;
      }
    }

    if (pin1_long == 3)
      pin1_long = 0;
      
    if (pin2_long == 2) {
      dist_l -= c_dist_divider_slower ;
      if (dist_l < 0) {
        dist_l = 0;
        pin2_long = 0;
      }
    }

    if (pin2_long == 3)
      pin2_long = 0;
}


//----------------------------------------------------------------
//  This function gets called about once per second, during the GPS
//  quiet time.  It's the best place to do anything that might take
//  a while: print a bunch of things, write to SD, send an SMS, etc.
//
//  By doing the "hard" work during the quiet time, the CPU can get back to
//  reading the GPS chars as they come in, so that no chars are lost.

static void getGPSData()
{
  //Can have "1-3 sats" but no location/no fix
  if (fix.valid.location)
    ready_to_go = true;
  else
    ready_to_go = false;
      
  if (fix.valid.satellites)
    sats = gps.sat_count;
  else
    sats = 0; //Would be 0 anyways

  //Should not display proper data (inform the user) until we get a stable GPS fix
  if (ready_to_go) 
    if (sats < 3)
      ready_to_go = false;
  
  //Speed is valid (and 0) if we have "some" GPS fix
  //If invalid it is 0 anyways, not need to check fix.valid.speed
  speed = fix.speed_metersph();
  
  //Heading = 0 and Invalid, if not moving -> hdg_valid = false
  hdg = fix.heading_cd(); //Always 0 if invalid
  if (fix.valid.heading) {
    hdg_valid = true;
  } else
    hdg_valid = false;

    
} // getGPSData()

void(* resetFunc) (void) = 0; //declare reset function @ address 0

static void initGPS() {
  gpsPort.attachInterrupt( GPSisr );
  
  //SwitchTo 115200
  gpsPort.begin( 9600 );

  //Disable not needed GPS Sentences
  gps.send_P(&gpsPort , F("PUBX,40,GSV,0,0,0,0,0,0") );
  delay( 250 );
  gps.send_P(&gpsPort , F("PUBX,40,GST,0,0,0,0,0,0") );
  delay( 250 );
  gps.send_P(&gpsPort , F("PUBX,40,ZDA,0,0,0,0,0,0"));
  delay( 250 );
  gps.send_P( &gpsPort , F("PUBX,40,VTG,0,0,0,0,0,0"));
  delay( 250 );
  
  //Go 5Hz
  sendUBX( ubxRate5Hz, sizeof(ubxRate5Hz) );
  delay( 250 );
  gps.send_P( &gpsPort , F("PUBX,41,1,3,3,115200,0"));
  gpsPort.flush();
  gpsPort.end();

  gpsPort.begin( 115200 );
  
}



static void powerOn() {
    display.begin();
    initDisplay();
  
    sprintf(display_buf,"   ON");
    display.showText(display_buf);
    display.update();
    initGPS();// Already has the delays
}

static void powerOff() {
   display.showIndicators(0);
   display.clear();
   display.update();
   
  sprintf(display_buf,"  OFF");
  display.showText(display_buf);
  display.update();

  //It will take interrupts here          
  sleep_enable();
  sleep_bod_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_cpu();

  //Wake Up
  sleep_disable();
}


static void powerStates() {

  bool inputHigh = FastGPIO::Pin<pwrPin>::isInputHigh();
  if (inputHigh == false) { //Battery Start -> Shut Down
    power_state = BAT_ON;
  } else {
    power_state = IGN_ON;
  }


//Power Actual Condition
//BAT_ON 1 //On battery
//IGN_ON 0 //On 12v

// Power Mode
//ON_PWR 0 //Is On
//OFF_PWR 1 //Is Off
//BATT_PWR 2 //Is On on Battery (1 -> 2)
//SW_ON_PWR 3 //Turning On (1,2) -> 2 -> 0
//SH_DOWN_PWR 4 //Turning Off (0,2) -> 3 -> 1

  if (power_mode == ON_PWR) {
    if (power_state == BAT_ON) {
      power_mode = SH_DOWN_PWR;
    }
    if (power_state == IGN_ON) //Optimization
      return;
  } else
  if (power_mode == OFF_PWR) {
    if (power_state == IGN_ON) {
      power_mode = SW_ON_PWR;
    } else if (power_state == BAT_ON) {//Was sleeping but ended here
      if ((pin1_long > 0)||(pin2_long > 0)) { //Middle button 
        power_mode = BATT_PWR;
      } else {
        power_mode = SH_DOWN_PWR;
      }
    }
  } else
  if (power_mode == BATT_PWR) {
    if (power_state == IGN_ON) {
      power_mode = SW_ON_PWR;
    }
    if (power_state == BAT_ON) {
      //PASS    
    }
  } else
  if (power_mode == SH_DOWN_PWR) {
    if (power_state == IGN_ON) {
      power_mode = ON_PWR; //No shutdown with ignition
    }
    if (power_state == BAT_ON) {
      power_mode = OFF_PWR;
      pin1_short = false;//Reset keys
      pin2_short = false;
      powerOff();
    }
  } else
  if (power_mode == SW_ON_PWR) {
    if (power_state == BAT_ON) {
      power_mode = BATT_PWR;
    }
    if (power_state == IGN_ON) {
      power_mode = ON_PWR;
      powerOn();
    }
  }
  
}

static void displayData()
{
  byte indic = 0;

  if (menu_mode == 1) {//Long press -> display reset
        if (display_mode == 0) {
          sprintf(display_buf,"ODO 0");
          display.showText(display_buf);
          display.update();
          return;
        }
        if (display_mode == 3) {
          sprintf(display_buf,"RESET");
          display.showText(display_buf);
          display.update();
          return;
        }
        if (display_mode == 2) {
          //DEBUG_PORT.print( F("User Off REQ!\n") );   
          sprintf(display_buf,"  OFF");
          display.showText(display_buf);
          display.update();
          return;
        }
  } else
  if (menu_mode == 2) {
    if (display_mode == 0) {
      dist_l = 0;
    }
    if (display_mode == 3) {//Reset
      resetFunc();
    }
    if (display_mode == 2) {//Off
        power_mode = SH_DOWN_PWR;//Requested to Turn Power Off
    }
    menu_mode = 0;
  }
  
  if (display_mode == 0)
    if (stopped == true)
      sprintf(display_buf,"-%2d%02d", i_dist_h, i_dist_l);//Was not working with unit32_t 
    else
      sprintf(display_buf," %2d%02d", i_dist_h, i_dist_l);//Was not working with unit32_t 
  else if (display_mode == 1)
    sprintf(display_buf,"S %3d",i_speed);
  else if (display_mode == 2) {
    char hv;
    if (hdg_valid == false)
      hv = '-';
    else
      hv = ' ';
    sprintf(display_buf,"H%c%3d",hv,i_hdg);
  } else if (display_mode == 3)
    sprintf(display_buf,"G  %02d",i_sats);

  if (i_sats == -1)
    indic = indic | 0;
  else if (ready_to_go == true)
    indic = indic | BATT_MEDIUM | BATT_HIGH;
  else
    indic = indic | BATT_HIGH;

  if ((display_mode == 0)||(display_mode == 5)) {
    if (c_odoGrade == 10)
      indic = indic | BATT_8;
    if (c_odoGrade == 100)
      indic = indic | BATT_32;//!!!! Does NOT work
  }
  display.showIndicators(indic);
  //ToDo: display depending on the Mode (HDG, SPD, ODO, DBG)
  display.showText(display_buf);
  display.update();

/*//DEBUG_PORT.print(F("MM "));
//DEBUG_PORT.print(menu_mode);
//DEBUG_PORT.print(F("DM "));
//DEBUG_PORT.print(display_mode);
//DEBUG_PORT.print(F("PM "));
//DEBUG_PORT.println(power_mode);*/


/*//DEBUG_PORT.print(F("AVG "));
//DEBUG_PORT.print(last5speeds);
//DEBUG_PORT.print(F(" 25 "));
//DEBUG_PORT.println(last25speeds);*/
/*  //DEBUG_PORT.print(F("P1 "));
  //DEBUG_PORT.print(pin1_long);
  //DEBUG_PORT.print(F(" P2 "));
  //DEBUG_PORT.print(pin2_long);
  //DEBUG_PORT.print(F(" LAST "));
  //DEBUG_PORT.print(lastDebounceTime_pin1);
  //DEBUG_PORT.print(F(" "));
  //DEBUG_PORT.println(lastDebounceTime_pin2);*/
  /*
  //DEBUG_PORT.print( F("Speed (Km/H): ") );
  //DEBUG_PORT.print( i_speed );
  //DEBUG_PORT.print( F(" valid: ") );
  //DEBUG_PORT.print( fix.valid.speed );
  //DEBUG_PORT.print( F(" Heading (Deg): ") );
  //DEBUG_PORT.print( i_hdg );
  //DEBUG_PORT.print( F(" valid: ") );
  //DEBUG_PORT.print( fix.valid.heading );
  //DEBUG_PORT.print( F(" Odo: ") );
  //DEBUG_PORT.print( dist_l );
  //DEBUG_PORT.print( F(" Display: ") );
  //DEBUG_PORT.print( display_buf );  
  //DEBUG_PORT.print( F(" HL: ") );
  //DEBUG_PORT.print( i_dist_h );
  //DEBUG_PORT.print( F(".") );
  //DEBUG_PORT.print( i_dist_l );
  //DEBUG_PORT.print( F(" Sats: ") );
  //DEBUG_PORT.print( i_sats );
  //DEBUG_PORT.print( F(" valid: ") );
  //DEBUG_PORT.print( fix.valid.satellites );

  //DEBUG_PORT.print( F(" display: ") );
  //DEBUG_PORT.print( display_mode );
  //DEBUG_PORT.print( F(" Ready: ") );
  //DEBUG_PORT.println(ready_to_go);

  //Visual Debug //WTF????
  if (ready_to_go == false) 
    FastGPIO::Pin<ledPin>::setOutputValueHigh();
  else
    FastGPIO::Pin<ledPin>::setOutputValueLow();*/
}
//------------------------------------
//  This is the main GPS parsing loop.

static void GPSloop()
{
 
  while (gps.available( gpsPort )) { //The code inside is executed at GPS Rate (c_odoHz)
    fix = gps.read();
    getGPSData();
    processData();
    processOdometer();
  }
  if (gps.overrun()) {
    gps.overrun( false );
    DEBUG_PORT.println( F("DATA OVERRUN: took too long to print GPS data!") );
  }

    processLongButtons();
    updateOdometer();
    checkLongPress();
    processButtons();
    powerStates();  
    displayData();

} // GPSloop

//--------------------------


void setup()
{

  // Another tweaks to lower the power consumption
  ADCSRA &= ~(1<<ADEN); //Disable ADC
  ACSR = (1<<ACD); //Disable the analog comparator


  initDisplay();
  
  DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT)
    ;

  DEBUG_PORT.print( F("OpenODO: started\n") );
  //DEBUG_PORT.print( F("  fix object size = ") );
  //DEBUG_PORT.println( sizeof(gps.fix()) );
  //DEBUG_PORT.print( F("  gps object size = ") );
  //DEBUG_PORT.println( sizeof(gps) );
  //DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );

  
  trace_header( DEBUG_PORT );
  DEBUG_PORT.flush();
  
  initGPS();
  DEBUG_PORT.print( F("Switched to 115200\n") );       

  greetDisplay();

   //Init Button Pins
  FastGPIO::Pin<bt1Pin>::setInputPulledUp();
  FastGPIO::Pin<bt2Pin>::setInputPulledUp();
  FastGPIO::Pin<pwrPin>::setInput(); 

  bool inputHigh = FastGPIO::Pin<pwrPin>::isInputHigh();
  if (inputHigh == false) { //Battery Start -> Shut Down
    power_state = BAT_ON;
    power_mode = SH_DOWN_PWR;
  }

  attachPCINT(digitalPinToPCINT(pwrPin), pwrInterrupt, CHANGE);
  attachPCINT(digitalPinToPCINT(bt1Pin), key1Interrupt, CHANGE);
  attachPCINT(digitalPinToPCINT(bt2Pin), key2Interrupt, CHANGE);
 

  //TODO: This loop is blocked if GPS is not connected
  while (gps.available( gpsPort ) == false) {//Still show something while waiting
    updateOdometer();
    processLongButtons();
    checkLongPress();
    processButtons();
    powerStates();
    displayData();
  }

  //First Fix
  fix = gps.read();
}

//--------------------------

void loop()
{
  GPSloop();
}
