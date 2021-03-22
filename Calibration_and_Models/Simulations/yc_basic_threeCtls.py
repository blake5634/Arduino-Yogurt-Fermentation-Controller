import numpy as np
import matplotlib.pyplot as plt
import control as ctl


# Define Task Seqeuence states
#
DENATURE = 0
COOL     = 1
FERMENT  = 2
COAST    = 3

tamb = 65  # ambient Temp deg F

Params = {}
Params['Pmax'] = 300
Params['Tdenature'] = 190 - tamb
Params['Tferment']  =  95 - tamb 
Params['Tamb'] = tamb

def tempfix(Y,tamb = 65):
    for i,y in enumerate(Y):
        Y[i] = y+tamb
    return Y

def ptarget(P, tmax, y):
   win = 0.02  # 2 percent Settling
   x=[0.0,tmax]
   y1 = (y+tamb)*(1-win)-tamb
   y2 = (y+tamb)*(1+win)-tamb
   ya  = tempfix([y,  y])
   yam = tempfix([y1,y1])
   yap = tempfix([y2,y2])
   P.plot(x,ya,   'r')
   P.plot(x,yam,  'r--',linewidth=1)
   P.plot(x,yap,  'r--',linewidth=1)

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

###########  Slow cooker model 1gal water ################
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

tmax = 300# min
nsamp = frame_rate * tmax
Texp = np.arange(0,tmax, dt)

th = np.zeros(np.shape(Texp))
for i in range(len(Texp)):
    th[i] = Texp[i] / 60.00          # time to hours
    

###########################################  Phase I
#
#    300W constant power until temp = Tdenature
#

if False:
#######  Simulation

    #  Power input from electric heating element
    Uexp = np.zeros(len(Texp))
    for i,u in enumerate(Uexp):
        Uexp[i] = Params['Pmax']

    #  Open Loop response of plant with no controller 
    t, y1, x = ctl.forced_response(ss_mod, Texp, [ Uexp ]) 


    ax,fig = plt.subplots() 
    y1 = tempfix(y1,Params['Tamb'])

    plt.plot(th,y1) #,th,Rmod)  # 

    ptarget(plt, th[-1], Params['Tdenature'])

    
    a = plt.gca()
    a.set_ylim([0,350])
    plt.title('Phase I: constant max pwr')



###########################################  Phase II
#
#    0 power until temp = Tferment

#  Power input from electric heating element

if True:
    Uexp = np.zeros(len(Texp))

    #  Open Loop response of plant with no controller 
    init_cond = Params['Tdenature']
    t, y2, x = ctl.forced_response(ss_mod, Texp, [ Uexp ], X0=init_cond)


    y2 = tempfix(y2,Params['Tamb'])
    ax,fig = plt.subplots() 
    #plt.plot(th,y4,th,yCEF,th,Cmod,th,Rmod)  # 
    plt.plot(th,y2) #,th,Rmod)  # 
    ptarget(plt, th[-1], Params['Tferment'])


    a = plt.gca()
    a.set_ylim([0,350])
    plt.title('Phase II: 0-power cool-down')
    a.set_xlabel('Time (hrs)')

##########################################  PID Control
#
#    PID to temp = Tferment
#

#MODE = 'Gains'  
MODE = 'Zeros'

rho = 1.25   #  regularization pole

if MODE == 'Gains':
    Kp = 2    #
    Ki = 0.1
    Kd = 80

    num = [1, Kp/Kd, Ki/Kp ]
#  OR
# design zeros of controller

if MODE == 'Zeros':
    z1 = 0.03
    z2 = 0.06
        
        
    ### anti windup zeros
    #z1 = 0.01
    #z2 = 0.06
    # reconstruct the gains
    Kd = 1500#  from the RL analysis
    Kp = Kd*(z1+z2)
    Ki = Kd*z1*z2
    
    print('Computed Gains: Kp: {} Ki: {} Kd: {}'.format(Kp,Ki,Kd))
    num = [rho, rho*(z1+z2), rho*z1*z2 ]  # loop gain/Kd for RL

# Denominator for both PID forms
den = [1, rho, 0] 

con = ctl.tf(num,den)
conpid = ctl.series(Kd, con)

olsys = ctl.series(con, ss_mod) # loop gain
ufb = ctl.tf(1,1)               # unity feedback
clsys = ctl.feedback( ctl.series(conpid, ss_mod), ufb, sign=-1) # negative fb closed loop  system
ctleff = ctl.feedback(conpid, ss_mod, sign=-1)  # control effort 

Rin = np.zeros(len(Texp))
for i,r in enumerate(Rin):
    Rin[i] = Params['Tferment']

init_cond = Params['Tferment'] * 0.8 -Params['Tamb']# not exactly on target
print('Closed Loop System definition')
print(clsys)

t, y3, x = ctl.forced_response(clsys, Texp, Rin , X0=[0,0,init_cond])
#t, y3, x = ctl.initial_response(clsys, Texp, Rin , X0=[0,0,init_cond])


y3 = tempfix(y3,Params['Tamb'])

ax,fig = plt.subplots() 
plt.plot(th,y3) #
ptarget(plt, th[-1], Params['Tferment'])

a = plt.gca()
a.set_ylim([85,105])
a.set_xlabel('Time (hrs)')
a.set_ylabel('T (deg F)')
plt.title('Phase III: Fermetation regulation')

plt.show()
