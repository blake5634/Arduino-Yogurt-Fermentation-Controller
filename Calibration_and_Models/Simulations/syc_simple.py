import numpy as np
import matplotlib.pyplot as plt
import control as ctl
import sys


###########    Modes
COOKMODE = 0
COOLDOWN = 1
FERMENT =  2

#
#   Test different system models
#   m1) Continuous time and discrete type pycontrol models:
#      m1a)  pyc model for ct plant
#      m1b)  pyc model for DT Plant
#      m1c)  pyc ct ctlr_PID
#      m1d)  pyc DT ctlr PID
#   m2) 'manual'/Object Oriented CT plant model
#   m3) 'manual'/Object Oriented CT PID controller
#   m4) 'manual'/Object Oriented DT PID controller
#
#  Closed Loop versions:
#     cl01)  pyctl closed loop CT
#     cl02)  pyctl closed loop DT
#     cl03)  manual/OO CT ctlr_PID
#     cl04)  manual/OO DT ctlr PID
#

tmax = 24*60  # minutes
tamb = 68     # ambient Temp deg F

# Default Parameters
Params = {}
# control parameters
Params['AntiWindup'] = True
Params['Emax'] = 5 # deg F
Params['Pmax'] = 300

Params['dt'] = 0.1     #simulation/estimation dt, minutes
Params['Ctldt'] = 1.0  # control sample time

Params['PWMtime'] = 5 #  time of one PWM cycle (min)
Params['EdotN'] = 50   # length of edot moving average
# correct ODE coeffs accounting for 
#     assumed power: 300W
#     actual power:  254.5W
Pheater = 254.5
cp = 300.0/Pheater
fudge = 1.045
fudge = 1.03
# PID Controller Gains
C1 = fudge*cp*1/340
C2 = fudge*cp*1/2260 
Params['Ca'] = (-C1-C2)
Params['Cb'] = C1
Params['Kp'] = 250.0
Params['Ki'] = 1.5
Params['Kd'] = 600.0

# Target Parameters
Params['Tdenature'] = 190
Params['Tferment']  =  105

#Simulation Parameters
Params['Tamb'] = tamb
Params['Tstart'] = tamb
Params['tmax'] = 24*60
Params['integrator'] = 'Euler'
#Params['integrator'] = 'Rectangular'  #untested

Params['Debug'] = False



def tempfix(Y,tamb = 65):
    return Y


# plot a step function target zone
def ptarget(P, tmax, y):
   win = 0.02  # 2 percent Settling
   x=[0.0,tmax]
   #y1 = (y+tamb)*(1-win)-tamb    # if simss are relative to tamb
   #y2 = (y+tamb)*(1+win)-tamb
   y1 = y*(1-win)                 # if sims are relative to 0.0
   y2 = y*(1+win)
   ya  = tempfix([y,  y])
   yam = tempfix([y1,y1])
   yap = tempfix([y2,y2])
   P.plot(x,ya,   'r')
   P.plot(x,yam,  'r--',linewidth=1)
   P.plot(x,yap,  'r--',linewidth=1)


plist = Params.keys()
numparms = 'Emax,Pmax,Tdenature,Tferment,Tamb,Tstart,dt,EdotN,Ca,Cb,Kp,Ki,Kd'.split(',')
boolparms = 'AntiWindup,Debug'.split(',')
strparms = 'integrator'

for i,a in enumerate(sys.argv):
    if a in plist:
        val = sys.argv[i+1]
        if a in numparms:
            Params[a] = float(val)
        if a in boolparms:
            if val == 'True':
                Params[a] = True
            else:
                Params[a] = False
        if a in strparms:
            Params[a] = val

#  must be int
Params['EdotN'] = int(Params['EdotN'])

ParmReport = 'Parameter Values: '
for p in plist:
    ParmReport += '\n {:20s} | {:}'.format(p,Params[p])
    
#
#  make sure proposed ICs match state vec.
def statematch(x,n,m,P = {}):
    valid = True
    try:
        if len(x) != n:
            valid = False
    except:
            valid = False
    if not valid:
        print ('Invalid State Vector '+m)
        quit()
    return

#   m4) 'manual'/Object Oriented DT PID controller
#

epsilon = 1.0E-7

class dt_PID:
    def __init__(self,Params):
        # PID design converted to DT difference eqn
        self.a  = np.array([2.328,  -23.28, 24.05])
        self.b  = np.array([0.3793, -1.379,  1.0 ])
        self.en = np.zeros(3)  # error history
        self.un = np.zeros(3)  # output history

    def set_gains(self,an,bn):
        for i,a in enumerate(an):
            self.a[i] = a
        for i,b in enumerate(bn):
            self.b[i] = b
            
        if b[2] != 1.0:
            print('dt_PID.set_gains(): invalid gains')
            quit()


    def update(self,t,P,temp, ref):
        ##  what about anti-windup in DT??
        if t%P['Ctldt'] < epsilon:  # is this a control sample?
            e = ref - temp   # negative unity gain feedback
            self.en[2] = self.en[1]
            self.en[1] = self.en[0]
            self.en[0] = e
            self.un[2] = self.un[1]
            self.un[1] = self.un[0]
            self.un[0] = 1/(self.b[0])*(self.a[2]*self.en[2]+self.a[1]*self.en[1]
                                        +self.a[0]*self.en[0]
                                        -self.un[2]-self.b[1]*self.un[1])
        else:
            pass

    def output(self,t,P):  # must call update just before.
        if t%P['Ctldt'] == 0:  # is this a control sample?
            u = self.un[0]
            if u > 300.0:  # physical limits of heater
                u = 300.0
            if u < 0.0:
                u = 0.0 
            self.un[0] = u  # heater saturation
            return u        # controller output
        else:
            return self.un[0]  #zero-order output hold
        
        
#   m3) 'manual'/Object Oriented CT PID controller
#
class PID:
    def __init__(self,P):
        self.kp = P['Kp']
        self.ki = P['Ki']
        self.kd = P['Kd']
        #state
        self.e1   = 0.0
        self.eint = 0.0
        # aux state for PID
        self.edot = 0.0
        self.edotbuf = np.zeros(P['EdotN'])
        self.i = 0
        self.ulast = 0  # sample and hold output
        
    def set_ICs(self,x,P):
        statematch(x,2,'for initial conditions',P)
        self.e1 = x[0]
        self.eint = x[1]
        
    def update(self,t,P,temp, ref):
        e = ref - temp   # negative unity gain feedback
        #print('error: {:4.1f}'.format(e))
        # use a moving average to smooth edot
        ed0 = (e-self.e1)/P['dt']
        sum = 0
        for i in range(P['EdotN']):  # most recent value is [0]
            j = P['EdotN'] - i - 2  # [N-2] --> [0]            
            sum += self.edotbuf[j]
            self.edotbuf[j+1] = self.edotbuf[j]
        self.edotbuf[0] = ed0
        sum += ed0
        self.edot = sum/P['EdotN']  # average of N first diffs
        
        if P['integrator'] == 'Euler':
            # Euler integration
            De  = e
        ## Euler
        #yEint[i] = yEint[i-1] + 0.5*dt*(y[i]+y[i-1])
        else: # Rectangular
            De = e
            
        if P['AntiWindup']:
            if abs(e) > P['Emax']:
                De = 0.0
                self.eint = 0.0
                
        self.eint += De    # integral of error
        # memory update        
        self.e1    = e     # previous error
        
    def report(self):
        self.i += 1
        print('--- PID:'.format(i))
        print('   eint: {:6.1f} edot: {:12.7f}  u: {:7.1f}'.format(self.eint, self.edot, self.ulast))
        print(' Ctl:  cp: {:6.1f} ci: {:6.1f} cd: {:6.1f}'.format(self.kp*self.e1,self.ki*self.eint,self.kd*self.edot))

    def output(self,t,P): 
        if t%P['Ctldt'] < epsilon:  # is this a control sample?

            u = self.kp*self.e1 +self.ki*self.eint +self.kd*self.edot
            if u > P['Pmax']:  # physical limits of heater
                u = P['Pmax']
            if u < 0.0:
                u = 0.0 
            self.ulast = u
        return self.ulast       # controller output
    
    
#   m2) 'manual'/Object Oriented CT plant model
class Cooker:
    def __init__(self,P):
        self.T = P['Tamb']
        #aux state
        self.Tm1  = 0.0
        self.tdot = 0.0
        self.tamb = 68.0  # ambient temp
        
    def set_ICs(self,x,P):
        statematch(x,1,'for Slow Cooker Model',P)
        self.T   = x[0]     # initial temperature
        self.Tm1 = x[0]   # previous temperature
        
    def update(self,P,pwr):
        if pwr < 0.0:
            print('pwr = {}: somethings wrong!'.format(pwr))
            quit()
        # Xdot = AX+BU
        Tdot = (P['Ca']*(self.T-tamb)  + P['Cb']*pwr) 
        self.tdot = Tdot # debug
        Tp1 = self.T + (Tdot * P['dt'])  # T <- T+dt*dT/dt 
        self.tp1 = Tp1 # debug
        if P['integrator'] == 'Euler':
            #Euler integration
            DT = P['dt'] * Tdot
        ## Euler
        #yEint[i] = yEint[i-1] + 0.5*dt*(y[i]+y[i-1])
        else:
            #Rectangular integration
            DT = Tdot*P['dt']
        
        # memory update
        self.Tm1 = self.T
        
        # perform the integration
        self.T += DT
        
    def report(self):
        print('-Therm')
        print('   T: {:6.3f}  Tm1: {:6.3f} tdot: {:6.3f} tp1: {:6.2f}'.format(self.T, self.Tm1,self.tdot,self.tp1))
        
    def output(self):
        return self.T


global pwmstate    # pwm state flag
global pwmt0       # time of pwm cycle start

pwmstate = 0
pwmt0 = 0

###################################################   convert power to PWM
def pwm(pwr, t, P):
    global pwmstate
    global pwmt0
    if t%P['PWMtime'] < epsilon:  # switching time
        pwmstate = 0
        pwmt0 = t     # log start time of PWM cycle
    else:
        pwmstate = 1
        
    if pwmstate == 0:
        pwmoutput = 1
    elif pwmstate == 1:
        pt = P['PWMtime']*pwr/P['Pmax'] # pwm ON time
        if t-pwmt0 < pt:
            pwmoutput = 1
        else:
            pwmoutput = 0
            
    return P['Pmax'] * pwmoutput




#
#      Simulation time range
#
#Tr = np.arange(0,12*60,Params['dt'])  # 24 hours
Tr = np.arange(0,Params['tmax'],Params['dt'])    # two hours
 
 
##############################################   Response arrays:
#
#
resp_m2 = np.zeros(len(Tr))     # 'manual'/Object Oriented CT plant model

resp_cl01 = np.zeros(len(Tr))   # pyctl closed loop CT
resp_cl02 = np.zeros(len(Tr))   # pyctl closed loop DT
ctleff_cl02 =np.zeros(len(Tr))

###########################################  Linear plant model

###########  Slow cooker model 1gal water 
#########    State Space with compartments

# Tdot = (-C1-C2)*T + C1*U
# Tdot =        a*T +  B*U

a = Params['Ca']
b = Params['Cb']

#absurdly simple state space model!!!
A = np.matrix(a)   # 1x1
B = np.matrix(b)
C = np.matrix(1)  # need this to see output
D = np.matrix(0.0)

# m1a:   pyctl plant model
ss_cook = ctl.StateSpace(A,B,C,D)
print('SS system pole(s): ', ss_cook.pole())

# m1b:  pyctl dt plant model
dt_cook = ss_cook.sample(Params['Ctldt'],'tustin') 

############################################## Closed Loop Continuous Time Linear System

#####################################################   PID Temperature Control
# PID Thermal Controller
z1 = 0.03   # 1/min
z2 = 0.06   # 1/min

# reconstruct the gains
Kd = 120#  from the RL analysis
Kp = Kd*(z1+z2)
Ki = Kd*z1*z2

print('pyctl Gains: Kp: {:12.6f} Ki: {:12.6f} Kd: {:12.6f}'.format(Kp,Ki,Kd))

# make a linear PID (with regularization pole rho)
rho = 15*z2
num = [rho*Kd, rho*Kp, rho*Ki]
den = [1, rho, 0]
#den = [1, 2*rho, rho*rho, 0]


##########################   PID Controller               (m1c)
#  Continuous time and pycontrol PID model (m1c)
ctl_lin_m1c = ctl.tf(num,den)
# make discrete time version of controller
dt_ctl_m1c = ctl_lin_m1c.sample(Params['Ctldt'], method='tustin')


#######################  try some other gains for OO PID ctl

Kp = Params['Kp']
Ki = Params['Ki']
Kd = Params['Kd']

print('  OO PID Gains: Kp: {:12.6f} Ki: {:12.6f} Kd: {:12.6f}'.format(Kp,Ki,Kd))

#   m3) 'manual'/Object Oriented CT PID controller
ctl_pid_m3 = PID(Params)

# unity feedback
ufb = ctl.tf(1,1)
# cl01) pyctl closed loop CT  
clsys_cl01 = ctl.feedback( ctl.series(ctl_lin_m1c, ss_cook), ufb, sign=-1) # negative fb closed loop  system

####   Test inputs for (cl101)

Utest = Tr.copy()
for i,u in enumerate(Utest):
    if Tr[i] < 10:
        Utest[i] = 0.0
    else:
        Utest[i] = Params['Tferment'] # closed loop ferm

#########################################################    pyctl step resp (cl01)
#t_cl01, resp_cl01 = ctl.step_response(clsys_cl01,Tr)

t_cl01, resp_cl01, X= ctl.forced_response(clsys_cl01, Tr, [Utest]) 
    
#for i,t in enumerate(Tr):
    #if i%20==0:
        #print ('{:5.1f} - {:4.2f}'.format(t,resp_cl01[i]))
#quit()
    

###############################################################   Plant   (m2)
#   m2) 'manual'/Object Oriented CT plant model
plant = Cooker(Params)


i=0
temp = Params['Tferment']-0.0
ref = Params['Tferment']
plant.set_ICs([temp], Params)


############################################## Closed Loop Discrete Time Linear System (cl02)
dt_ufb = ufb.sample(Params['Ctldt'],'tustin')             # unity feedback
# cl02) pyctl closed loop DT
dt_clsys = ctl.feedback( ctl.series(dt_ctl_m1c, dt_cook), dt_ufb, sign=-1) # negative fb closed loop  system
#ctleff = ctl.feedback(conpid, ss_cook, sign=-1)  # control effort 


#print('Discrete time controller (m1d) (Tustin):')
#print(dt_ctl_m1c)


#####################################################
#
# set up "nice" initial conditions
#

einit = ref-temp
edinit = einit + 10.26
# set initial 'prev' error and initial eint.
 # m2)
plant.set_ICs([Params['Tstart']], Params)
plant.set_ICs([ 78], Params)

##############################################################    Object DT control  (m4)
oo_dtctl = dt_PID(Params)

# kickstart plant model before controller
# m2)
#plant.update(Params,0.0)  # 0 power input

################################################################  Object oriented Model Simulations

################################## Simulate CT Object-based System  m2) and cl02) 
#Params['Debug']=True
print('Initial cond.')
print('Plant: ref: {:}, T: {:},  Tm1: {:}'.format(ref,plant.T,plant.Tm1))
for t in Tr:
    plant.update(Params, 0)  # plant response
    temp = plant.output()
    
    if False:
        print('\n t = {:5.2f} '.format(t), end='')
        #ctl_pid.report()
        plant.report()
        x = input('Enter to continue:')
    
    resp_m2[i] = plant.output()
    i+=1
        
######################################################        cl04)  manual/OO DT ctlr PID
##
## set up "nice" initial conditions (again)
##

tstart = Params['Tstart']
tref  = 75
einit = tref-tstart
## set initial 'prev' error and initial eint.
#oo_dtctl.set_ICs([einit,0.0],Params) # state is previous error
plant.set_ICs([Params['Tstart']], Params)

################################## Simulate DT Object-based System
#     cl04)  manual/OO DT ctlr PID

resp_cl04   = np.zeros(len(Tr))
ctleff_cl04 = np.zeros(len(Tr))

print('Initial cond.')
print('ref: {:}, T: {:},  e: {:}'.format(ref,plant.T,ref-plant.T))
i = 0
temperature = temp
for t in Tr:
    oo_dtctl.update(t,Params,temperature,ref)
    heater_power = oo_dtctl.output(t,Params)
    plant.update(Params,heater_power)
    temperature = plant.output()
    
    #plant.update(Params, 250)
    if False:
        print('\n t = {:5.2f}'.format(t))
        ctl_pid.report()
        plant.report()
        x = input('Enter to continue:')
    
    resp_cl04[i] = temperature
    ctleff_cl04[i] = heater_power
    i+=1

   
######################################################        cl03)  manual/OO CT ctlr PID
##
## set up "nice" initial conditions (again)
##

tstart = Params['Tstart']
tref  = Params['Tferment']
einit = tref-tstart
## set initial 'prev' error and initial eint.
ctl_pid_m3.set_ICs([einit,0.0],Params) # state is previous error
plant.set_ICs([Params['Tstart']], Params)
print('Plant IC check: plant.T = ',plant.T, '(set to: ', Params['Tstart'],')')

################################## Simulate CT Object-based System    (cl03)
#     cl03)  manual/OO CT ctlr PID

resp_cl03   = np.zeros(len(Tr))
ctleff_cl03 = np.zeros(len(Tr))
err_cl03    = np.zeros(len(Tr))

Tgoal = Params['Tferment']

print('Initial cond.')
print('ref: {:}, T: {:},  e: {:}'.format(Tgoal,plant.T,ref-plant.T))
heater_power = 0.0  # start with off
temperature = Params['Tstart']
mode = COOKMODE  # start with max power up to Tdenature
for i,t in enumerate(Tr):
    #  Plant
    plant.update(Params,heater_power)
    temperature = plant.output()
    
    #################################################  Control depending on MODE
    if mode == COOKMODE:
        heater_power = Params['Pmax']
        if temperature > Params['Tdenature']:
            mode = COOLDOWN
            
    if mode == COOLDOWN:
        heater_power = 0
        if temperature < Params['Tferment'] * 1.0:
            mode = FERMENT
            
    if mode == FERMENT:
        #  PID Controller with PWM output
        ctl_pid_m3.update(t,Params,temperature,Tgoal)
        desired_power = ctl_pid_m3.output(t,Params)
        ##  create PWM with 5min cycle
        heater_power = pwm(desired_power, t, Params)
    
    #plant.update(Params, 250)
    if Params['Debug']:
        print('\n t = {:5.2f}'.format(t))
        ctl_pid_m3.report()
        #plant.report()
        x = input('Enter to continue:')
    
    resp_cl03[i]   = temperature
    ctleff_cl03[i] = 25* heater_power/Params['Pmax']  # better readability with PWM
    err_cl03[i]    = temperature - Tgoal

##############################################
##############################################  Plotting
##############################################

################# pyctl closed loop CT  (cl01)
if False:
    ax,fig = plt.subplots()
    plt.plot(Tr,resp_cl01,'b')
    ptarget(plt,Tr[-1],Params['Tferment'])
    a = plt.gca()
    pltZoom = True
    if pltZoom:
        pass
    else:
        a.set_ylim([0,2.0])
        a.set_xlim([0,Tr[-1]])
    plt.title('pyctl CT closed loop PID step response (cl01)')

if True:
#
#   OO - CT Closed Loop     
#
#   cl03)  manual/OO CT ctlr PID
    ax,fig = plt.subplots()
    plt.plot(Tr,resp_cl03,'b', Tr, ctleff_cl03,'r',Tr,err_cl03,'.g')
    ptarget(plt,Tr[-1],Params['Tferment'])
    a = plt.gca()
    a.set_ylim([-100,350])
    a.set_xlim([0,Tr[-1]])
    a.set_xlabel('Time (min)')
    a.set_ylabel('Temperature (degF)')
    a.grid(True)
    #plt.title('Manual/OO Cont Time Closed Loop (cl03)')
    plt.title('Complete 3-Stage cycle')

#
#   DT OO closed loop
#
if False:
    ax,fig = plt.subplots()
    plt.plot(Tr,resp_cl04, 'b', Tr,ctleff_cl04,'r') 
    a = plt.gca()
    a.set_ylim([-100,350])
    a.set_xlim([0,Tr[-1]])
    plt.title('Object DT PID control Test')

ParmReport += '\n\n'
print(ParmReport)

print(' "Steady state error = ',err_cl03[-1])

plt.show()
