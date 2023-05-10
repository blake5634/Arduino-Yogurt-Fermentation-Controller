/*
 *  Thermister Calibration test code   May '23
 *
 *   Goal: identify corrections to the interpolation-based resistance to temp
 *      for the white sensor
 *
    Yogurt fermentation controller
    (C) Blake Hannaford 2021
    Revised: use eeprom to store state for power loss / blackout recovery (4/23)

 */


//#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

//    CONTROLLER MODES
#define COOKMODE  0
#define COOLDOWN  1
#define FERMENT   2

// State storage
#define STATE_ADDR 1  // address in EEPROM where state is stored (as 1 byte)
// Estimate temp when needed for state change
#define STATE_TEMP_TIME  1.0        // minutes of averaging (there's a thermal transient
                                 // when cold milk poured into cooker
#define STATE_TEMP_SAMP_MIN  3  // temp samples per minute

//   PID COMMANDS
#define  UPDATE   0
#define  OUTPUT   1
#define  INIT     2

//    CONTROLLER PARAMETERS
#define ANTIWINDUP  1    // https://en.wikipedia.org/wiki/Integral_windup
#define Emax       1.5   // max error integral value (deg F) (only integrate SMALL errors)
#define Pmax     300.0   // power of the heating element
#define Tamb      68.0   // ambient temp (deg F)
#define DT         0.5   // minutes (for ctl and estimator updates)
#define Ctldt      1.0   // ctl OUTPUT period, minutes
#define PWMtime    2.0   // pwm period, minutes: 6 sec for testing 2min quieter
#define EdotN      10      // number of avg pts for edot estimate
#define DISP_periodsec  5  // update disp every 5 sec.

// only for plant simulation
#define PLANT_Ca        -0.00410825
#define PLANT_Cb         0.0035710

/* orig 2020
#define Kp        250
#define Ki        1.5
#define Kd        600.
*/

// with new periods April 2023:

#define Kp       30.00  // based on full power if e > 10
#define Ki         .6
#define Kd        20.0

// simple time conversions
static long sec2ms=1000, min2sec=60;
static long min2ms = 60*1000;

//     YOGURT MAKING PARAMETERS

#define Tdenature 195.0
#define Tferment  110.0  // new preferred temp?

//      HARDWARE PARAMETERS

#define LED_bd     LED_BUILTIN // Arduino pro mini
#define LCDi2c      0x27
#define LCDROWS        2
#define LCDCOLS       16

#define WHITESENSOR 0  // two different thermistors
#define BLACKSENSOR 1

/*
 *    Set these to zero so we can measure needed corrections
 */
#define SENSOR_OFFSET_WHITE   0.0  //  Empirical: add this to computed temp (minimize error at Tferment)
#define SENSOR_CORRECTION_DENATURE_WHITE  0.0  //calib adjust for high temps



#define RELAY_Socket01 7
#define RELAY_Socket02 8
#define HEAT_ON        0
#define HEAT_OFF       1

// we use char arrays for text. not Strings()
static char str[LCDCOLS];    // a buffer for sprintf(str,xxxxx)
static char ch_arr_01[LCDCOLS];
static char ch_arr_02[LCDCOLS];
static char  *modename;      // four char name for displaying current state (frmrly mode)



// set up the LCD driver for display
LiquidCrystal_I2C lcd(LCDi2c, LCDCOLS, LCDROWS);

/*
 * Display on top line (16 chars max)
 * */
int disp(char *str){    
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(str);
    return(0);
    }
    
int line2(char *str){
    lcd.setCursor(0,1);
    lcd.print(str);
    return(0);
}


// the setup routine runs once when you press reset:
void setup() {
  //Serial.begin(9600);

  /* initialize the display to blank with the backlight on
  */
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Hello Yogurt");
  lcd.setCursor(0,1);
  lcd.print("      Fans!");
  delay(2500);

  // initialize the digital pin as an output.
  pinMode(LED_bd, OUTPUT);
  pinMode(RELAY_Socket01, OUTPUT);
  digitalWrite(RELAY_Socket01, HIGH); // OFF: relays are active-LOW

}

int ain() {
    int AnalogPin = A0;
    int data = analogRead(AnalogPin);
    return data;
//    return 583 ;  // stub this for ADC(T=95F)
}

float readResistance(){
    float Vcc = 5.0;
    float R1 = 5000.0;
    float Vin = (5.0/1023.0) * float(ain());
    float Rth = R1 / ((Vcc/Vin)-1.0);
    return Rth  ;
}

float  R2T(float r, int sensor) {//interpolation fit of temperature vs. R
    //int nintpts =  19 ;
    //float  tarray[] = { 44.0, 53.0, 62.0, 71.1, 81.1, 92.0, 102.7, 121.1, 138.0, 140.0, 145.8, 152.0, 153.0, 165.0, 166.0, 178.0, 178.0, 184.8, 185.0 };
    //float  rarray[] = {
    // 21100.0, 17400.0, 14100.0, 11200.0, 8700.0, 7000.0, 5500.0, 3740.0, 2500.0, 2430.0, 2170.0, 1890.0, 1850.0, 1460.0, 1450.0, 1160.0, 1140.0, 1000.0, 985.0 };
    float  rth_white[] = {25000.0000, 19460.0000, 14450.0000, 11470.0000, 10480.0000,
7780.0000, 5340.0000, 3840.0000, 2290.0000, 1410.0000,
 115.0000};

    float rth_black[] = {30000.0000, 21370.0000, 15250.0000, 11700.0000, 10440.0000,
7420.0000, 4760.0000, 3240.0000, 1740.0000,  953.0000,
 115.0000};

    float tarray[] = {  33.1000,   45.8000,   59.2000,   70.6000,   73.7000,
  88.4000,  107.7000,  125.8000,  156.8000,  194.6000,
 225.0000};

    //float  rarray[11]; // allocate mem
    float *p;  // pointer to correct resistance vals.
    int nintpts = 11;
    if (sensor == WHITESENSOR)
        //memcpy(rth_white,rarray,sizeof rarray);
        p = rth_white;
    if (sensor == BLACKSENSOR)
        //memcpy(rth_black,rarray,sizeof rarray);
        p = rth_black;
    float tval = -1.0;   // flag for bad lookup
    for (int i=0; i< nintpts; i++){
        if (r > *(p+i)) {
           // float  dTdR = (tarray[i]-tarray[i-1])/(rarray[i]-rarray[i-1]);
           // tval = tarray[i] + ((r-rarray[i])) * dTdR;
            float dTdR = (tarray[i]-tarray[i-1])/( *(p+i)- *(p+i-1) );
            tval = tarray[i] + ((r-*(p+i))) * dTdR;
            break;
            }
    }
    // SENSOR_OFFSET_WHITE is an empirical factor to zero error at Tferment with "white" thermistor
    if (sensor == WHITESENSOR) {
        /*
         *  Empirical thermistor corrections
         *   1) SENSOR_OFFSET_WHITE    a constant offset to correct temp at t=Tferment (where PID operates)
         *   2) hack_highT             gradually increasing correction for higher temps > Tferment (for accurate COOK)
         */
        float retval = 0.0;
        retval = tval + SENSOR_OFFSET_WHITE;  // correction 1)
        float hack_highT = 0.0;
        if (retval > Tferment)  // only gradually correct, above Tferment
            hack_highT = (tval-Tferment)*(SENSOR_CORRECTION_DENATURE_WHITE/(Tdenature-Tferment));  // correct +8degF too high reading at Tdenature
        retval += hack_highT;   // correction 2)
        return(retval);
        }
    else return(float(tval)); // no corrections yet for black sensor
    }

    /*
     *
     *      loop()
     *
     */
void loop() {
    long int tsec = 0;  // elapsed time
    long int tmin = 0;
    long int tms = 0;
    float r1, temperature;
    float sum;

    //   what time is it? (time since start in absolute units)
    tms     = (long int) millis();
    tsec    = (long int)(float(tms)/1000.0);
    tmin    = (long int)(tsec/60);

    sum = 0.0;
    for (int i=0;i<5;i++){   // acquire and avg R value of thermistor
        r1 = readResistance();
        sum += r1; }
    r1 = sum/5.0;
    temperature = R2T(r1 , WHITESENSOR);

    /*
     * Display
     */

    sprintf(str,"T:%s  R:%s",  dtostrf(temperature,5,1,ch_arr_02), dtostrf(r1,6,0,ch_arr_01));
    disp(str);
    int sec = (int)(tsec - (long int)(60*tmin));
//       modename.toCharArray(ch_arr_01,sizeof(ch_arr_01));
    // add current PWM ratio to display
    sprintf(str,"%02d:%02d", (int)tmin, sec );
    line2(str);

    delay(500);
    }
