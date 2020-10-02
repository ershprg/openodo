

#include <LEDDisplayDriver.h>

// This module is tested with a specific HT1621 module with 6 digits and background light
// Manual for library: http://lygte-info.dk/project/DisplayDriver%20UK.html
// By HKJ from lygte-info.dk

#ifndef _HT1621_6D_
#error "_HT1621_6D_ must be defined in LEDDisplayDriver.h for this sketch to work"
#endif

//*************************************************************************
// Define the pins used for the display connection
#define DATA_PIN A0
#define WR_PIN A1
#define CS_PIN A2

LEDDisplayDriver display(DATA_PIN, WR_PIN,CS_PIN,true,6);	// 6 digit display

void setup() {
}

byte hello[] = {digitH, digitE, digitL, digitL, digitO, digitEmpty};

void loop() {

  // Start with full battery
  display.showIndicators(BATT_HIGH | BATT_MEDIUM | BATT_LOW);

  // Say hello
  display.showDigits(hello, 0, 6);
  delay(3000);

  // A number
  display.showNum(1234);
  delay(3000);

  // A bit of the battery is used
  display.showIndicators(BATT_MEDIUM | BATT_LOW);

  // In hex
  display.showHex(1234);
  delay(3000);

  // Show number on part of display, clear also affect indicators
  display.clear();
  display.showIndicators(BATT_MEDIUM | BATT_LOW);
  display.showNum(4.1234567, 2, 4);
  delay(3000);

  // Use fixed decimals to stay where there is points on the display
  display.showNum3decimals(22.1234);
  delay(3000);

  // Battery is getting low
  display.showIndicators(BATT_LOW);

  // Show some text and a counting mumber
  display.showDigit(digitT, 0);
  display.showDigit(digiti, 1);
  unsigned long t = millis();
  while (millis() < t + 15000) {
    display.showNum((millis() - t) / 1000.0, 2, 4);
  }

  // Nothing left in the battery
  display.showIndicators(0);
  display.showMinus(0, 6);
  delay(5000);

}