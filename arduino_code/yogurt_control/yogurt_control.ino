/*
    Yogurt fermentation controller
    (C) Blake Hannaford 2021
    Revised: use eeprom to store state for power loss / blackout recovery (4/23)

    Note: be sure to compile for correct clock speed or millis() will be off!
    (current board ardino pro mini 16Mhz)

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
static long int sec2ms=1000;
static long int min2sec=60;

//     YOGURT MAKING PARAMETERS

#define Tdenature 195.0
#define Tferment  107.0

//      HARDWARE PARAMETERS

#define LED_bd     LED_BUILTIN // Arduino pro mini
#define LCDi2c      0x27
#define LCDROWS        2
#define LCDCOLS       16

#define WHITESENSOR 0  // two different thermistors
#define BLACKSENSOR 1
/*
 *   no longer needed but enabled to compile old R2T()
 *
 */
 
#define SENSOR_OFFSET_WHITE   2.0  //  Empirical: add this to computed temp (minimize error at Tferment)
#define SENSOR_CORRECTION_DENATURE_WHITE  -8.0  //calib adjust for high temps

#define RELAY_Socket01 6   
#define RELAY_Socket02 7
#define HEAT_ON        0
#define HEAT_OFF       1

// we use char arrays for text. not Strings()
static char str[LCDCOLS];    // a buffer for sprintf(str,xxxxx)
static char ch_arr_01[LCDCOLS]; 
static char ch_arr_02[LCDCOLS]; 
static char  *modename;      // four char name for displaying current state (frmrly mode)

 
// some state variables
static float r1,power,temperature, set_point_temp;
  
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

// Global state variable
int Gstate = COOKMODE;

// this is how we make state transitions
void changetostate(int st) {
  Gstate = st;
  char tmpcodedst = (char) (st + 3);  //  so zero is not valid we just add 3 to state
  EEPROM.update(STATE_ADDR, tmpcodedst);  // save current state
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

  /*
   * 
   *    Detect power loss during a cook cycle and restart in correct mode
   * 
   */

  int nsamp = 0;
  // When we wake up after a normal batch concludes, we need to get back 
  // from "FERMENT" to state "COOK" (0).   
  //  Also need this if there is some glitch or error.
  //  Key is we need to change to COOK state if milk is COLD (below ambient) because
  //  that is most likely a new batch from fridge.
  disp("Chkg milk st:");
  float sum = 0.0, r1=0.0, tmptemp=0.0;
  int tmp = 100 * (int) (10 * STATE_TEMP_TIME);   // perserve decimal and convert to ms
  int loopcnt =  4;
  
  for (int i=0;i<loopcnt;i++){   // acquire and avg R value of thermistor
        nsamp++;
       // sprintf(str, "%2d  %d   ",nsamp,del);
       // line2(str); 
        r1 = readResistance();
        sum += r1;
        sprintf(str,"%02d  T: %d F ", nsamp, int(R2Tnew(r1,WHITESENSOR)) ) ;
        line2(str);    
        delay(1500);
      }
  tmptemp = R2Tnew(sum/nsamp, WHITESENSOR);
  
  // regardless of EEPROM state:
  if (tmptemp < Tamb and tmptemp > 0.0) {  // unplugged sensor = -INF
        changetostate(COOKMODE);  // if milk is cold we must have meant COOK!
        set_heater(HEAT_ON);
        temperature = tmptemp;
        }
  else {
        // Check eeprom for a stored state.  We **might** be waking up from
        // a (brief??) power loss.
        int eeprom_state = int(EEPROM.read(STATE_ADDR));
        eeprom_state -= 3;  //"decode" them memory value
        if (eeprom_state < COOKMODE || eeprom_state > FERMENT) {
            eeprom_state = COOKMODE; // if invalid memory value, default to COOK
            }
        changetostate(eeprom_state);
        }
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



float R2Tnew(float r, int sensor) {
    // measured resistance values (last two pts are "fake" for better end-interpolation
    float p[] = {29400, 13000, 10920, 7565, 5080, 4680, 2400, 2080, 1530, 1105, 1100};
    //measured temp values
    float tarray[] = {32, 65, 74, 93, 113, 118, 157, 166, 184, 207, 213};
    int nintpts = 11;
    float minr = 1100.0;
    float maxr = 29400.0;
    float minT = 32.0;
    float maxT = 213.0;
    float tval = -1.0;

    if (sensor == WHITESENSOR) {
        if (r > maxr) return minT;
        if (r < minr) return maxT;
        for (int i = 0; i < nintpts; i++) {
            if (r >= p[i]) {
                float dTdR = (tarray[i] - tarray[i-1]) / (p[i] - p[i-1]);
                tval = tarray[i] + (r - p[i]) * dTdR;
                break;
            }
        }
    } else {
        disp("Error:R2Tnew()");
        while (1) ;
        }

    return tval;
}

/*   Disable for testing R2Tnew()
 *    
 
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
        //
         //  Empirical thermistor corrections
         //  1) SENSOR_OFFSET_WHITE    a constant offset to correct temp at t=Tferment (where PID operates)
         //  2) hack_highT             gradually increasing correction for higher temps > Tferment (for accurate COOK)
        
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
*/

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
long int pwmt0 = 0;
int pwmoutput = 0;
long int pt = 0;

//  PWM toggling function
int pwm_tog(float pwr, long int tsec, long int ptms, long int pwm_periodsec){
    //
    //   pwr:  controller output (Watts)
    //   tsec:  current time (seconds)
    //   ptms:   current time (ms)
    //   pwm_periodsec:  PWM signal period (sec)
    //   pt:    pwm ON time (ms)
    //
    static long int nexttime_pwm;
    static long int nextoff_pwm;
    if (tsec > nexttime_pwm){ 
        // schedule start of next 
        nexttime_pwm += pwm_periodsec;
        pwmt0 = ptms;  // store the start time of PWM cycle (ms)
        // lock the pwm value for this period
        if (pwr > Pmax) pwr = Pmax;
        if (pwr < 0.0)  pwr = 0.0;
        pt = (1000.0*((float)pwm_periodsec)*pwr/Pmax); // pwm ON time
        nextoff_pwm = pwmt0 + pt;
        }    
    // note pwm value can change during one PWM period!
    if (ptms < nextoff_pwm) {
        pwmoutput = HEAT_ON ;
        }
    else {
        pwmoutput = HEAT_OFF; 
        }
    return(pwmoutput);    
}


int set_heater(int command){ // >=1:ON, 0:OFF
     static int state=HEAT_OFF; 
     if(!(command==state)){  // only do IO if state change
        if (command == HEAT_ON){
            digitalWrite(RELAY_Socket01,  LOW);  // TURN ON
            state = HEAT_ON;
            }
        else {
            digitalWrite(RELAY_Socket01, HIGH);  // TURN OFF
            state = HEAT_OFF;
            }
//         state = status;
//         }
    return(0);
    }
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
            //eint = 0.0;
            //  Preset the integral term b/c steady state value is ~12%
            eint = 0.12 / Ki;
            edot = 0.0;
            for (int i=0;i<EdotN;i++){
                edotbuf[i] = 0.0;
                }
            e1 = 0.0;
            return(-1);
            }
            
        case UPDATE: {
            int rval;  // flag to signal type of update result
            rval = 0;
            float sum;
            // edotbuf[0] == oldest
            // edotbuf[EdotN-1] == most recent
            // sum up and shift back the edot buffer
            sum = edotbuf[0];
            for (int i=0;i<(EdotN-1);i++){ 
                sum += edotbuf[i+1];
                edotbuf[i] = edotbuf[i+1];
                }
            edotbuf[EdotN-1] = (e-e1)/DT; // DT in minutes
            edot = sum/(float)EdotN;
            e1 = e;
            
            //  Antiwindup feature
            if (ANTIWINDUP && (abs(e)>Emax)){
                // do not integrate if error is large.
                // eint = 0.0;  // kill the integrator for large e
                rval = -2;
                } 
            else {
                eint += e * DT;   // integrate if error is small
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

 /***********************************************************************
  *
  *
  *       The main loop
  *
  */ 
void loop() {
  // don't add delays(!) 
  float x=0;
  float a=0;
  float y=0;  
  long int tsec = 0;  // elapsed time
  long int tmin = 0;
  long int tms = 0;
  int t_hour  = 0;
  
  // text (char buffers, not String()s)
  
  static long int  nexttime_estim = 0;
  static long int  nexttime_ctl = 0;
  static long int  nexttime_disp = 0;
    
  unsigned long int ptms ;
  
  // update period for estimator
  static long int est_periodsec   = (long) int(min2sec*DT);   //  estimation period (sec)
  
  // Update period for controller
  static long int ctl_periodsec = (long) int(min2sec*Ctldt);  //  control period   (sec)

  // PWM output signal period
  static long int pwm_periodsec = (long) int(PWMtime*float(min2sec)); //
   
  //   what time is it? (time since start in absolute units)
  tms     = (long int) millis();
  tsec    = (long int)(float(tms)/1000.0);
  tmin    = (long int)(tsec/60);
  t_hour  = tmin/60;

  //  are we alive?? flash LED
  if (tsec % 2 < 1)
    digitalWrite(LED_bd,HIGH);
  else
    digitalWrite(LED_bd,LOW);

  // Do "state estimation" and "controller state update"
  if (tsec > nexttime_estim) {//  measure current temperature
    nexttime_estim += est_periodsec;  // schedule next time
    float sum;
    sum = 0.0;
    for (int i=0;i<5;i++){   // acquire and avg R value of thermistor
        r1 = readResistance();
        sum += r1; } 
    temperature = R2Tnew(sum/5.0 , WHITESENSOR);
    if(Gstate==FERMENT) {
        PID(temperature, Tferment, UPDATE);  //  update edot etc.
        }
    }
  
  // Detect and execute control output 
  if (tsec > nexttime_ctl) {
    nexttime_ctl += ctl_periodsec; 
    switch (Gstate){
        case COOKMODE:// 0
            modename = "Cook";
            set_heater(HEAT_ON);
            power = Pmax;
            set_point_temp = Tdenature;
            if (temperature > Tdenature) {
                changetostate(COOLDOWN);
                }
            break;   
            
        case COOLDOWN:  // 1
            modename = "Cool";
            set_heater(HEAT_OFF);
            set_point_temp = Tferment;
            power = 0.0;
            if (temperature <= Tferment) {
                PID(temperature, set_point_temp, INIT);
                changetostate(FERMENT);
                }
            break;  
            
        case FERMENT:   // 2
            modename = "Ferm";
            set_point_temp = Tferment;
            // generate controller output
            power = PID(temperature, set_point_temp, OUTPUT);
            // output of PID power via PWM is coded just below:
            break;  
            
        }
  }

  // Drive the heater on and off to deliver needed power
  // operate the PWM cycle quickly every loop cycle (in ferment state only)
  //  but do this AFTER power is updated in control loop
  if (Gstate==FERMENT) {
    // parameters of pwm_tog control switching times.
    int u_binary = pwm_tog((float)power,tsec,tms, pwm_periodsec);
    set_heater(u_binary);  // physical output only occurs on changes.
    }
  
 //  update display
if(tsec > nexttime_disp){
        nexttime_disp += DISP_periodsec;
        sprintf(str,"T:%s S: %s",  dtostrf(temperature,5,1,ch_arr_02), dtostrf(set_point_temp,3,0,ch_arr_01));
        disp(str); 
        int dmin = tmin - 60*t_hour;
  //      int sec = (int)(tsec - (long int)(60*tmin));
 //       modename.toCharArray(ch_arr_01,sizeof(ch_arr_01));
        // add current PWM ratio to display
        sprintf(str,"%02d:%02d %4s %3d%%", t_hour, dmin, modename, int(100*power/Pmax));
        line2(str);  
        }
}  // end of loop()
 
