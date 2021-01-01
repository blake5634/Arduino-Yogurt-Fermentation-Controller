/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
//#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

//    CONTROLLER MODES
#define COOKMODE  0
#define COOLDOWN  1
#define FERMENT   2

//   PID COMMANDS
#define  UPDATE   0
#define  OUTPUT   1
#define  INIT     2

//    CONTROLLER PARAMETERS
#define ANTIWINDUP  1
#define Emax  5.0
#define Pmax 300.0
#define Tamb      68.0
#define DT         0.10
#define Ctldt      1.00
#define PWMtime    5.0
#define EdotN      50

#define PLANT_Ca        -0.00410825
#define PLANT_Cb         0.0035710

#define Kp        250
#define Ki        1.5
#define Kd        600.0

//     YOGURT MAKING PARAMETERS

#define Tdenature 190.0
#define Tferment  105.0

//      HARDWARE PARAMETERS

#define LED_bd        13
#define LCDi2c      0x27
#define LCDROWS        2
#define LCDCOLS       16

#define RELAY_Socket01 7
#define RELAY_Socket02 8
#define HEAT_ON        0
#define HEAT_OFF       1

// Pin 13 has an LED connected on most Arduino boards.
// give it a name: 

LiquidCrystal_I2C lcd(LCDi2c, LCDCOLS, LCDROWS);

// the setup routine runs once when you press reset:
void setup() {  
  Serial.begin(9600);
  // This will attempt to initialize the display to blank with the backlight on 
 
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("YC Testing:");
  lcd.setCursor(0,1);
  lcd.print(" Closed loop sim.:");
  delay(760);

  
  // initialize the digital pin as an output.
  pinMode(LED_bd, OUTPUT);
  pinMode(RELAY_Socket01, OUTPUT);
  digitalWrite(RELAY_Socket01, HIGH); // OFF: relays are active-LOW

  //////////    TESTING setups
  
  int u1 = PID(100.0,100.0, INIT);
  disp("PID Init");
  delay(500);
  
  tempsim(300.0, INIT);
  disp("Plant Sim Init");
  delay(200);
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
 

 
/***************************************************************
 * 
 *    Testing: new code modules for full system control
 * 
 */

int pwmstate = 0;
long pwmt0 = 0;
int pwmoutput = 0;
int pt = 0;
//  PWM toggling function
int pwm_tog(float pwr, long tms , long pwmper_ms){
    if ((tms%pwmper_ms)==0){
        pwmt0 = tms;  // log start time of PWM cycle
        }
    pt = (float)pwmper_ms * pwr/Pmax; // pwm ON time
    if (tms-pwmt0 < pt) {
        pwmoutput = 1 ;
        }
    else {
        pwmoutput = 0; 
        }
    return(pwmoutput);    
}


int set_heater(int status){ // >=1:ON, 0:OFF
    if (status >= 1){
        digitalWrite(RELAY_Socket01, LOW);  // TURN ON
        }
    else {
        digitalWrite(RELAY_Socket01, HIGH);  // TURN OFF
        }
    return(0);
    }
    
int PID(float T, float goal, int cmd){
    static float eint;   // integral of error
    static float e1;     // last value of error
    static float edot;   // dE/dt
    static float edotbuf[EdotN];  // window for moving avg
    // compute error (negative unity fb)
    float e = goal-T;  
    switch (cmd){
        case INIT: {
            eint = 0.0;
            edot = 0.0;
            for (int i=0;i<EdotN;i++){
                edotbuf[i] = 0.0;
                }
            e1 = 0.0;
            return(-1);
            }
            
        case UPDATE: {
            int rval;
            float sum;
            // edotbuf[0] == oldest
            // edotbuf[EdotN-1] == most recent
            // sum up and shift back the e buffer
            sum = edotbuf[0];
            for (int i=0;i<(EdotN-1);i++){ 
                sum += edotbuf[i+1];
                edotbuf[i] = edotbuf[i+1];
                }
            edotbuf[EdotN-1] = (e-e1)/DT; 
            edot = sum/(float)EdotN;
            e1 = e;
            
            //  Antiwindup feature
            if (ANTIWINDUP && (abs(e)>Emax)){
                eint = 0.0;  // kill the integrator for large e
                rval = -2;
                } 
            else {
                eint += e;   // integrate if error is small
                rval = -1;
                } 
            return(rval);
            }
            
        case OUTPUT: {
            float u = Kp*e + Ki*eint + Kd*edot;
            /*Serial.print("controller: ");
            Serial.print(e,1);
            Serial.print(Kp*e,1);
            Serial.print(eint,1);
            Serial.print(Ki*eint,1);
            Serial.print("\n");
            delay(20);
            */
            if (u>Pmax)  u = Pmax;
            if (u<0.0)   u = 0.0;
            return(int(u));
        }
        
    }
        
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
    
    
float tempsim(int power, int cmd){
    static float T;
    static float Tm1;
    float Tdot;
    switch (cmd){
        case INIT: {
            T = Tamb;
            Tm1 = T;
            Tdot = 0.0;
            return(-1);
            }
            
        case UPDATE: {
            Tdot = PLANT_Ca*(T-Tamb) + PLANT_Cb*power;
            T += Tdot*DT;
            return(T);
            }
        case OUTPUT: {
            return(T);
            }      
    
       }
    }

/*
 * 
 *  End of new code
 * 
 * ***********************************************************/
float t=0;
void loop() {  
  int tsec = 0;  // elapsed time
  int tmin = 0;
  int thr  = 0;
  char  str[LCDCOLS];
  char numstr[LCDCOLS]; 
  char numstr2[LCDCOLS]; 
  long tms = 0;
//  tms = millis();
  
  long sec2ms=1000, min2sec=60;
  long pwmperiod_ms = 0;
  
  t += DT;   // minutes
  tms += ((long)DT)*min2sec*sec2ms;  
  
  float  tt,temp_test = 99.80;
  float  goal_test = 100.0;
  // Things we can't initialize each looptime
  static float temperature;
  static int u = 0;
  static int uu = 0;
  /*
   * Test of PWM function
   * 
   
  //int PID(float T, float goal, int cmd){
 
  int u2 = PID(temp_test, goal_test, UPDATE);
  sprintf(str,"u: %3d",u2);
  disp(str);
  delay(500);
  int u3 = PID(temp_test, goal_test, UPDATE);
   u3 = PID(temp_test, goal_test, UPDATE);
   u3 = PID(temp_test, goal_test, UPDATE);
  
   u3 = PID(temp_test, goal_test, OUTPUT);
  sprintf(str,"u: %3d",u3);
  disp(str);
  delay(2000);  */
  
  /*
   *   Test of closed loop with cooker simulator
   */
  if (t < 5.0) {   // initial conditions
      u = 100;
      tempsim(u,INIT);
      temperature = tempsim(u,OUTPUT);
      }
  else{
    uu = PID(temperature, goal_test, UPDATE);
    u  = PID(temperature, goal_test, OUTPUT);
    }
  Serial.print("t: ");
  Serial.print(t,1);
  Serial.print(" T: ");
  Serial.print(temperature, 1);
  Serial.print(" e: ");
  Serial.print(goal_test-temperature,1);
  Serial.print(" u: ");
  Serial.println(u,DEC);
  
  temperature = tempsim(u, UPDATE);
  
  tt = t/5.0;
  if (fabs(tt-(float)int(tt)) < DT){
    sprintf(str,"t:%s T:%s", dtostrf(t,5,1,numstr), dtostrf(temperature,5,1,numstr2));
    disp(str); 
    sprintf(str,"e: %s", dtostrf(temperature-goal_test,5,1,numstr2));
    line2(str);
    delay(1);
    }
  
  
  /*
  //
  //  Test of Relay control
  //
  digitalWrite(LED_bd, HIGH);
  set_heater(HEAT_ON);
  delay(1000);
  digitalWrite(LED_bd, LOW);
  set_heater(HEAT_OFF);
  delay(1000); 
  */
  
  /*
  // Test of PWM function
  pwmperiod_ms = (long)(1) * min2sec * sec2ms;
  if (t%10 == 0){ 
        //  Testing:
        pwmperiod_ms = 100;
        int u = pwm_tog(100,t,pwmperiod_ms);
        set_heater(u);
        sprintf(str,"%3d  %s", t, ltoa(u,numstr,10));// pwm_tog(100.0,t,0.1)) ;
        dsp(str);
        }
        
  */
  delay(10);
  
   
}

