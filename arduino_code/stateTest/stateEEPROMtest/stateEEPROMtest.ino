/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.
 */
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

//    CONTROLLER MODES
#define COOKMODE  0
#define COOLDOWN  1
#define FERMENT   2

#define LCDi2c      0x27
#define LCDROWS        2
#define LCDCOLS       16

LiquidCrystal_I2C lcd(LCDi2c, LCDCOLS, LCDROWS);

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);
  // This will attempt to initialize the display to blank with the backlight on

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Hello Yogurt");
  lcd.setCursor(0,1);
  lcd.print("      Fans!");
  delay(200);
  // Erase
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");
}

int Gstate = COOKMODE;

unsigned long tms ;
long tsec = 0;  // elapsed time
long tmin = 0;
int thr  = 0;

long nexttimesec = 1;
int intervalsec = 10;
int tinstatesec = 0;
int tsecp = 0;


void loop() {
    //   what time is it? (time since start in absolute units)
    long t;
    tms = millis();
    tsec = long(float(tms)/1000.0);
    if (tsec > tsecp) {
      //  Serial.print(tsec);
        tinstatesec++;
        tsecp = tsec;
        }

    if (tsec > nexttimesec) {
        //  LCD
        Serial.print("Tsec: ");
        Serial.print(tsec);
        Serial.print(" tinstatesec: ");
        Serial.print(tinstatesec);
        Serial.print(" State: ");
        Serial.print(Gstate);Serial.print("\n");
        lcd.setCursor(0,0);
        lcd.print("State:");
        lcd.setCursor(8,0);
        lcd.print(Gstate);
        lcd.setCursor(8,0);
        lcd.setCursor(0,1);
        lcd.print("Time:");
        lcd.setCursor(8,1);
        lcd.print(tinstatesec); 
        nexttimesec += 1;
    }
    //  State Machine
    if (tinstatesec > 8) {
        if (Gstate != 2) tinstatesec = 0;
        if (Gstate == 0) Gstate = 1;
        else if (Gstate == 1) Gstate = 2;
        else if (Gstate == 2) Gstate = 2;  // just stick in state 3
    }

}
