- Arduino Nano (328P)
- Neo6 GPS
- HT1621 5-digit LCD
- ICO control switch

Power Input:
2-Pin M8 Connector -> DC-DC In (see the real ICO/RNS device)

Power:
DC-DC OUT + -> Arduino VN
DC-DC OUT- -> Arduino GND
DC-DC OUT+ -> (via Schottky Diode)   Arduino D2 (Battery Detect)

9v Bat + -> (via Schottky Diode or it EXPLODES)  Arduino VN
9v Bat - -> Arduino GND

LCD
Arduino D12 -> LCD WR 
Arduino D13 -> LCD CS
Arduino D7 -> LCD Data

ICO: 
Arduino A0 -> ICO Up
Arduino A1 -> ICO Down

GPS:
Arduino RX0 -> GPS TX
Arduino TX1 -> GPS RX

Arduino Voltage OUT:
Arduino +5V -> LCD Vcc
Arduino +5V -> GPS VCC

Arduino GND OUT:
Arduino GND -> LCD Gnd
Arduino GND -> GPS Gnd
Arduino GND -> ICO Gnd

Controls:
ICO Up:
    Short: +1
    Long: ++++

ICO Down:
    Short: -1
    Long: ----

Middle button:
    Short: Switch Mode
    Long:
       ODO Mode: Reset ODO

Display Modes:
x.xx : Odometer
S   1  : Speed
H   1  : Active Heading
H-  1  : Heading (current heading undetermined)
G   5  : 5 Satellites