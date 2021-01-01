import numpy as np
import matplotlib.pyplot as plt
import control as ctl

# Slow Cooker Data
f = open('data.csv','r')
ti = []
tmp = []
for l in f:
    d = l.split(',')
    ti.append(int(d[2]))
    tmp.append(int(d[3]))
    
#for i,tim in enumerate(ti):
    #print(i, tim, tmp[i])
#quit()

# Slow Cooker Model

frame_rate = 1.0  # frames/min
dt = 1.0/(frame_rate*10)  # 10 samples per min.

tc1 = 200  # minutes
pwr = 300      # Watts
tamb = 60   # ambient Temp deg F

####### Transfer Function
num =  1.0/tc1
den = [1, 1/tc1]

#wn = 2.0 * 6.28 
#zet = 0.6

#num = [wn*wn]
#den = [1, 2*zet*wn, wn*wn]
model = ctl.tf(num,den)
dt_system = ctl.sample_system(model, dt, method='tustin')
#print(model)
##t, y = ctl.step_response(model)
#mag, ph, w = ctl.bode(model)


#########    State Space with compartments

# Tdot = (-C1-C2)*T + C1*U
# Tdot =        a*T +  B*U

# initial estimate 

C1 = 1/310
C2 = 1/2260

a = (-C1-C2)
b = C1

#absurdly simple state space model!!!
A = np.matrix(a)   # 1x1
B = np.matrix(b)
C = np.matrix(1)  # need this to see output
D = np.matrix(0.0)
ss_mod = ctl.StateSpace(A,B,C,D)
print('SS system pole(s): ', ss_mod.pole())

#######  Simulation
tmax = 500# min
nsamp = frame_rate * tmax
Texp = np.arange(0,tmax, dt)

#  Power input from electric heating element
Uexp = np.arange(0,tmax, dt)  #  Heater on High
for i,u in enumerate(Uexp):
    if u < 181:            # 181 is exper time heater was shut off
        Uexp[i] = pwr    
    else:
        Uexp[i] = 0  # ambient temp
        
        
Rexp = np.arange(0,tmax, dt)  #  hack: Rexp(t) = t initially
for i,t in enumerate(Rexp):
    if t < 181:            #  desired temp rel to ambient
        Rexp[i] = 185-tamb    # denature temp.
    else:
        Rexp[i] = 94-tamb     # fermentation temp
        
#  Open Loop response of plant with no controller
#t,y = ctl.step_response(dt_system,Texp)
#t,y = ctl.input_output_response(model, Texp, Uexp)
#t, y, x = ctl.forced_response(model, Texp, [ Uexp ])
t, y, x = ctl.forced_response(ss_mod, Texp, [ Uexp ])
#t, y = ctl.step_response(ss_mod, Texp)
 
#for i,x in enumerate(t):
    #if i%100 == 0:
        #print ('{:6.1f} {:12.6f}'.format(x, y[i]))
#quit()

##########################################  Control

#MODE = 'Gains'  
MODE = 'Zeros'

rho = 0.73   #  regularization pole
if MODE == 'Gains':
    Kp = 2    #
    Ki = 0.1
    Kd = 80

    num = [1, Kp/Kd, Ki/Kp ]
#  OR
# design zeros of controller

if MODE == 'Zeros':
    z1 = 0.0055
    z2 = 0.01
        
    # more aggressive zeros
    z1 = 0.04
    z2 = 0.08
    
    
    ### anti windup zeros
    #z1 = 0.01
    #z2 = 0.06
    # reconstruct the gains
    Kd = 1176.0 #  from the RL analysis
    Kp = Kd*(z1+z2)
    Ki = Kd*z1*z2
    
    print('Computed Gains: Kp: {} Ki: {} Kd: {}'.format(Kp,Ki,Kd))
    num = [rho, rho*(z1+z2), rho*z1*z2 ]  # loop gain/Kd for RL

# System Setups both PID forms
den = [1, rho, 0] 

con = ctl.tf(num,den)
conpid = ctl.series(Kd, con)

olsys = ctl.series(con, ss_mod) # loop gain
ufb = ctl.tf(1,1)
clsys = ctl.feedback( ctl.series(conpid, ss_mod), ufb, sign=-1) # negative fb closed loop  system
ctleff = ctl.feedback(ctl.series(conpid, ss_mod), ufb, sign=-1)  # control effort
clsysSat = ctl.feedback( ctl.series(conpid, ss_mod), ufb, sign=-1) # negative fb closed loop  system

##############################################   Simulations
#
#  Run Linear simulations and analysis
#
#kr = np.arange(0,0.1,0.0002)
#ctl.root_locus(olsys,krange=kr,xlim=(-0.02,0.0),ylim=(-0.01,0.1),grid=True)
ctl.root_locus(olsys,xlim=(-0.1, 0.01),ylim=(-.1,0.1),grid=True)
#ctl.root_locus(olsys,grid=True)

if False:
    # compute closed loop response to ref input Rexp
    t1,y1,x1 = ctl.forced_response(clsys, Texp, [ Rexp ])
    print('DC Gain: {}'.format(ctl.dcgain(clsys)))

    # compute the control effort (Watts)
    t1,yce, xce = ctl.forced_response(ctleff, Texp, [Rexp])

    for i,y5 in enumerate(y1):
        y1[i] += tamb   # add back ambient temp
        
    #ax,fig = plt.subplots() 
    #plt.plot(t1,y1)  # x-values
    #a = plt.gca()
    #a.set_ylim([50,200])
    #plt.title('Closed loop response')

    ##########  Linear Systems Plotting
    t2 = np.zeros(np.shape(t))
    for i in range(np.shape(t)[0]):
        y[i] = tamb + y[i]         # add back in ambient
        t2[i] =  float(i)*dt
        
     
    ax,fig = plt.subplots() 
    plt.plot(t2,y,ti,tmp,t1,y1,t1,Rexp)  # x-values
    a = plt.gca()
    a.set_ylim([50,200])
    plt.title('Data vs Model')
        

    ax,fig = plt.subplots() 
    plt.plot(t,yce)  # x-values
    a = plt.gca()
    a.set_ylim([0,1000])
    plt.title('Control Effort')
    
    
#
#   Non - Linear
#
    
########################################## Input-Output Heater
# with saturation

# heater update
def hupdate(t,x,u, params={}):  # basic saturation
    Pmax = 300  # 300 watt heater
    # t = time (min) (not used)
    # u = controller power signal
    # returns heater power after saturation
    power = x[0]
    if u < 0.0:
        u = 0
    if u > Pmax:     #300Watt heating element
        u = Pmax
    dP = u-power
    return dP
    
# https://python-control.readthedocs.io/en/0.8.3/iosys.html
#  Heater system
heater_IO = ctl.NonlinearIOSystem(
    hupdate,None, 
    inputs='p', 
    outputs='ph',
    states='p')

ctl_PID_IO = ctl.iosys.tf2io(conpid)  # includes Kd
plant_IO = ctl.LinearIOSystem(ss_mod)
ufb_IO = ctl.iosys.tf2io(ufb)         # unity feedback

clsysSat = ctl.feedback( 
    ctl.series(ctl_PID_IO,heater_IO,plant_IO), 
    ufb_IO,
    sign=-1) # negative fb closed loop  system w/ Saturation

# Saturation test:
#input = Texp  # simple ramp
#for i,iN in enumerate(input):
    #input[i] = 2*iN - 250 # expose the saturation
# saturation only
#t4,y4 = ctl.input_output_response(heater_IO, Texp, input)

# closed loop non linear response (experimental)
#t4,y4 = ctl.input_output_response(clsysSat, Texp, [ Rexp ])

if False:
    dt = 1 #     min
    Tmod = np.arange(0,24*60,dt)  # 24 hours by minutes
    Rmod = np.zeros(len(Tmod))
    Cmod = Rmod
    for i,t in enumerate(Tmod):
        if t <=300:
            Rmod[i] = 188-tamb  # denature phase
            Cmod[i] = 188
        else:
            Rmod[i] = 94-tamb   # fermentation
            Cmod[i] = 94


    # closed loop non-linear response (model)
    t4,y4 = ctl.input_output_response(clsysSat, Tmod, [ Rmod ])

    # closed loop non-linear control effort (model)
    clsysEFF = ctl.feedback( 
        ctl.series(ctl_PID_IO,heater_IO),plant_IO,
        sign=-1) # negative fb closed loop  system w/ Saturation
    t5,yCEF = ctl.input_output_response(clsysEFF, Tmod, [ Rmod ])

    th = np.zeros(np.shape(t4))
    for i in range(np.shape(t4)[0]):
        #y4[i] = tamb + y4[i]         # add back in ambient
        th[i] = t4[i] / 60.00    # time to hours
        
    ax,fig = plt.subplots() 
    #plt.plot(th,y4,th,yCEF,th,Cmod,th,Rmod)  # 
    plt.plot(th,y4,th,yCEF,th,Cmod) #,th,Rmod)  # 
    a = plt.gca()
    a.set_ylim([0,350])
    plt.title('closed loop with SATURATION')


    ########################################## 
    #
    #   Time dependent controller
    #

    #  1) full power until T=185   (about 3 hrs)
    #  2) 0 power cooldown until T=94 (about 2 hrs)
    #  3) PID Controller    T=94
    #
    #  State vector:
    #     x[0] = temperature (relative to tambient)
    #     x[1] = PID integrator

    # Define Task Seqeuence states
    #
    DENATURE = 0
    COOL     = 1
    FERMENT  = 2
    COAST    = 3

    parms = {}
    params['Pmax'] = 300
    params['Kp'] = Kp
    params['Ki'] = Ki
    params['Kd'] = Kd
    params['Thermal_a'] = a
    params['Thermal_b'] = b
    params['Tdenature'] = 190 - tamb
    params['Tferment']  =  95 - tamb
    params['TaskState'] = DENATURE
    params['Tdot'] = 0

    # heater update

    def tdc_update(t,x,u, params={}):  # basic saturation
        # t = time (min) (not used)
        # u = thermal system state (temp)
        # State x is *controller* state
        #     x[0] = temperature above ambient
        #     x[1] = deriv of temp
        #     x[2] = PID integrator
        #     x[3] = PID e-dot
        
        mode = params['TaskState']
        Pmax = params['Pmax']
        curTemp = x[0]
        
        if mode == DENATURE:
            pwrout = Pmax     # Heat as fast as possible
            if CurTemp >= params['Tdenature']:
                params['TaskState'] = COOL
            
        if mode == COOL:
            pwrout = 0.0      # Cool down as fast as possible
            if CurTemp <= params['Tdenature']:
                params['TaskState'] = FERMENT        
        
        if mode == FERMENT:  # final capture mode (we don't leave)
            # PID Controller for ferm temp.
            goal = params['Tferment']
            pwrout = params['Kp']* (goal-x[0]) + \
                    params['Ki']* x[1] + \
                    params['Kd']* edot
            
        ###
        #
        #  Compute state update
        #
        xd = np.zeros(len(x))  # return this 
        # Tdot comes from thermal model (should come from system state)
        xd[0] = params['Thermal_a']*x[0] + params['Thermal_b']*ctlpwr
        # evolve PID controller 
        xd[1] = (goal-x[0])  #  error integrator
        
        ctlpwr = params['Kp'] * (goal-x[0]) + \
                params['Ki'] * x[1] +\
                params['Kd'] * xd[2]
                
            
        if mode == COAST:
           pass 
        
        temp = x # Current temperature
        power = x[0]
        if u < 0.0:
            u = 0
        if u > Pmax:     #300Watt heating element
            u = Pmax
        dP = u-power
    

    # https://python-control.readthedocs.io/en/0.8.3/iosys.html
    #  Heater system
    tdc_IO = ctl.NonlinearIOSystem(
        tdc_pdate,tdc_output, 
        dt = 0,            # continuous time
        inputs=[] ,
        outputs='ph',
        states=[temp,ctl01, ctl02])



#ax,fig = plt.subplots()
#plt.plot(ti,tmp)
plt.show()
