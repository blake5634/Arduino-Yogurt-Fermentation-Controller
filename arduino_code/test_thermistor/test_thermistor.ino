/*
  
Test version your thermocouple cable for shorts etc.
  
 */
//#include <Software//Serial.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>



//      HARDWARE PARAMETERS

#define LED_bd        13  // on-board LED
#define LCDi2c      0x27
#define LCDROWS        2
#define LCDCOLS       16

#define THERMISTOR_PIN  A1
#define VREF_PIN        A0


LiquidCrystal_I2C  lcd(LCDi2c, LCDCOLS, LCDROWS);

int ain() {
    int data = analogRead(THERMISTOR_PIN);
    return data;
//    return 583 ;  // stub this for ADC(T=95F)
}
int vref_ain() {
    int data = analogRead(VREF_PIN);
    return data;
}

float readResistance(){
    float Vcc = 5.0;
    float R1 = 5000.0;
    float Vin = (5.0/1023.0) * float(ain());
    float Rth = R1 / ((Vcc/Vin)-1.0);
    return Rth  ; 
}

int WHITESENSOR = 0;
int BLACKSENSOR = 1;

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
    return(float(tval)); 
    }

float R2TV1(float r, int sensor) {//interpolation fit of temperature vs. R
    int nintpts =  19 ;
    float  tarray[] = { 44.0, 53.0, 62.0, 71.1, 81.1, 92.0, 102.7, 121.1, 138.0, 140.0, 145.8, 152.0, 153.0, 165.0, 166.0, 178.0, 178.0, 184.8, 185.0 };
    float  rarray[] = {
     21100.0, 17400.0, 14100.0, 11200.0, 8700.0, 7000.0, 5500.0, 3740.0, 2500.0, 2430.0, 2170.0, 1890.0, 1850.0, 1460.0, 1450.0, 1160.0, 1140.0, 1000.0, 985.0 };
 
    float tval = -1.0;
    for (int i=0; i< nintpts; i++){
        if (r >= rarray[i]) {
            float dTdR = (tarray[i]-tarray[i-1])/(rarray[i]-rarray[i-1]);
            tval = tarray[i] + ((r-rarray[i])) * dTdR;
            break;
            }
    }
    return(tval); 
    }
    

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
    
int flag = 1;  // for first time through

// the setup routine runs once when you press reset:
void setup() {
  //Serial.begin(9600);
  // This will attempt to initialize the display to blank with the backlight on

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Thermistor Test App");
  lcd.setCursor(0,1);
  lcd.print("   Hello   ");
  delay(2500);

  pinMode(LED_bd, OUTPUT);  // on board led

}

void loop() {
  digitalWrite(LED_bd, HIGH);
  delay(250);
  digitalWrite(LED_bd, LOW);
  delay(250);
  float x=0;
  float a=0;
  float y=0;  
  long tsec = 0;  // elapsed time
  long tmin = 0;
  int thr  = 0;  
  
  // text (char buffers, not String()s)
  static char str[LCDCOLS];
  static char ch_arr_01[LCDCOLS]; 
  static char ch_arr_02[LCDCOLS]; 
  static char  *modename;

  float sum;
  int navgR = 5;
  sum = 0.0;
  // Get thermistor resistance
  for (int i=0;i<navgR;i++){   // acquire and avg R value of thermistor
    sum += readResistance();
    delay(100); }
  float Ravg = sum/navgR ;
  //  Convert to temperature
  float temperature = R2T(Ravg , WHITESENSOR);
  //  Get vref ADC count
  int VrefCnt = vref_ain();
 //  update display
  sprintf(str,"R: %s Vref:",  dtostrf(Ravg,7,1,ch_arr_02));
  disp(str);
//       modename.toCharArray(ch_arr_01,sizeof(ch_arr_01));
// add current PWM ratio to display
  sprintf(str,"T: %s %d",  dtostrf(temperature, 7, 1, ch_arr_01), VrefCnt);
  line2(str);

}  // end of loop()
 
