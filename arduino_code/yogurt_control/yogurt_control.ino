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
#define Ctldt      1
#define PWMtime    5.0   // minutes
#define EdotN      50
#define DISP_periodsec  5  // update disp every 5 sec.

#define PLANT_Ca        -0.00410825
#define PLANT_Cb         0.0035710

#define Kp        250
#define Ki        1.5
#define Kd        600.0

//     YOGURT MAKING PARAMETERS

#define Tdenature 195.0
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
  delay(500);
  
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
    
    
 
/***************************************************************
 * 
 *    Testing: new code modules for full system control
 * 
 */

int pwmstate = 0;
long pwmt0 = 0;
int pwmoutput = 0;
long pt = 0;
//  PWM toggling function
int pwm_tog(float pwr, long tsec, long tms, long pwm_periodsec){
    //
    //   pwr:  controller output (Watts)
    //   tsec:  current time (seconds)
    //   tms:   current time (ms)
    //   pwm_periodsec:  PWM signal period (sec)aa
    //
    static long nexttime_pwm; 
    if (tsec > nexttime_pwm){ 
        // schedule start of next 
        nexttime_pwm += pwm_periodsec;
        pwmt0 = tms;  // store the start time of PWM cycle (ms)
        }    
    pt = (long)(1000.0*((float)pwm_periodsec)*pwr/Pmax); // pwm ON time
    if (tms-pwmt0 < pt) {
        pwmoutput = HEAT_ON ;
        }
    else {
        pwmoutput = HEAT_OFF; 
        }
    return(pwmoutput);    
}


int set_heater(int status){ // >=1:ON, 0:OFF
//     static int state; 
//     if(!(status==state)){  // only do IO if state change
        if (status == HEAT_ON){
            digitalWrite(RELAY_Socket01,  LOW);  // TURN ON
            }
        else {
            digitalWrite(RELAY_Socket01, HIGH);  // TURN OFF
            }
//         state = status;
//         }
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

void loop() { 
  float x=0;
  float a=0;
  float y=0;  
  long tsec = 0;  // elapsed time
  long tmin = 0;
  int thr  = 0;  
  
  // text and String
  static char str[LCDCOLS];
  static char ch_arr_01[LCDCOLS]; 
  static char ch_arr_02[LCDCOLS]; 
  static String modename;
  
  static long  nexttime_estim, nexttime_ctl, nexttime_disp;
  static long sec2ms=1000, min2sec=60;
  
  // update period for estimator
  static int est_periodsec   = int(min2sec*DT);   //  estimation period (sec)
  
  // Update period for controller
  static int ctl_periodsec = int(min2sec*Ctldt);  //  control period   (sec)

  // PWM output signal period
  static long pwm_periodsec = PWMtime*min2sec; //
  
//   Serial.println("periods: est, ctl, pwm: (sec)");
//   Serial.print(est_periodsec);
//   Serial.print(ctl_periodsec);
//   Serial.print(pwm_periodsec);
//   Serial.println(' ');
  
  // some state variables
  static float r1,power,temperature;
  
  static int mode;  // which process stage?  (COOKTIME etc).
  
  unsigned long tms ;
 
  delay(500);  //burn most of 1 seconds for ~1sec loop update
  
  //   what time is it? (time since start in absolute units)
  tms = millis();
  tsec = long(float(tms)/1000.0);
  tmin = long(tsec/60.0);
  thr  = int(tmin/60.0); 
  
  // kick off Cook Mode:
  if(tsec < 3) {
      mode = COOKMODE;
      modename = "COOK";
      set_heater(HEAT_ON);
      } 
  // Do "state estimation" and "controller state update"
  if (tsec > nexttime_estim) {//  measure current temperature
    nexttime_estim += est_periodsec;  // schedule next time  

            //  are we alive??
            digitalWrite(LED_bd,HIGH);
            delay(48);
            digitalWrite(LED_bd,LOW);
            delay(2);
    float sum;
    sum = 0.0;
    for (int i=0;i<5;i++){   // acquire and avg R value of thermistor
        r1 = readResistance();
        sum += r1; } 
    temperature = R2T(sum/5.0 , WHITESENSOR);
    if(mode==FERMENT) {
        PID(temperature, Tferment, UPDATE);  //  update edot etc.
        }
    }
  static float tgoal = Tdenature;
  
  // Detect and execute control output 
  if (tsec > nexttime_ctl) {
      nexttime_ctl += ctl_periodsec ; 
    switch (mode){
        case 0:// COOKMODE:               
            modename = "Cook";
            set_heater(HEAT_ON);
            tgoal = Tdenature;
            if (temperature > tgoal) {
                mode = COOLDOWN;
                }
            break;   
            
        case 1: //COOLDOWN: 
            modename = "Cool";
            set_heater(HEAT_OFF);
            tgoal = Tferment;
            if (temperature < Tferment) {
                PID(temperature,Tferment, INIT);
                mode = FERMENT;
                }
            break;  
            
        case 2: //FERMENT:  
            modename = "Ferm";
            tgoal = Tferment;
            // generate controller output
            power = PID(temperature, Tferment, OUTPUT); 
            // convert power to pwm.
            int u_binary = pwm_tog((float)power,tsec,tms, pwm_periodsec);
            set_heater(u_binary);
            break;  
            
        }
  }
 //  update display
if(tsec > nexttime_disp){
        nexttime_disp += DISP_periodsec;
        sprintf(str,"T:%s S: %s",  dtostrf(temperature,5,1,ch_arr_02), dtostrf(tgoal,3,0,ch_arr_01));
        disp(str); 
        int min = tmin - 60*thr;
        int sec = (tsec - (long)(60.0*(float)tmin));
        modename.toCharArray(ch_arr_01,sizeof(ch_arr_01));
        // add current PWM ratio to display
        sprintf(str,"%02d:%02d %s %2.1f", thr, min, ch_arr_01, pwr/Pmax);
        line2(str);  
        }

}  // end of loop()
 
