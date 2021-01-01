import numpy as np
import numpy.polynomial.polynomial as npp
import matplotlib.pyplot as plt
import control as ctl


def tempfix(Y,tamb = 68):
    for i,y in enumerate(Y):
        Y[i] = y+tamb
    return Y

def ptarget(P, tmax, y):
   win = 0.02  # 2 percent Settling
   x=[0.0,tmax]
   y1 = (y+tamb)*(1-win)-tamb
   y2 = (y+tamb)*(1+win)-tamb
   ya  = tempfix([y,  y],tamb)
   yam = tempfix([y1,y1],tamb)
   yap = tempfix([y2,y2],tamb)
   P.plot(x,ya,   'r')
   P.plot(x,yam,  'r--',linewidth=1)
   P.plot(x,yap,  'r--',linewidth=1)


##########################################   Read in data
# Slow Cooker Data
f = open('milkdata.csv','r')
# time, etmin, temp F, rNTC (Kohms)

ti = []
tmp = []
rth = []

for l in f:
    d = l.split(',')
    ti.append(int(d[1])) # elapsed min
    tmp.append(float(d[2])) # temp in deg F
    rth.append(1000.0*float(d[3]))  # r in ohms (from KOhm)
    

###########  Slow cooker model 1gal water ################
#########    State Space with compartments

# Tdot = (-C1-C2)*T + C1*U
# Tdot =        a*T +  B*U

# initial estimate 

# correct these accounting for 
#     assumed power: 300W
#     actual power:  254.5W
Pheater = 254.5
cp = 300.0/Pheater
fudge = 1.045
fudge = 1.03
C1 = fudge*cp*1/340
C2 = fudge*cp*1/2260

a = (-C1-C2)
b = C1

#absurdly simple state space model!!!
A = np.matrix(a)   # 1x1
B = np.matrix(b)
C = np.matrix(1)  # need this to see output
D = np.matrix(0.0)
ss_mod = ctl.StateSpace(A,B,C,D)

dt = 1
tmax = 350# min
Texp = np.arange(0,tmax, dt)
Uexp = np.arange(0,tmax, dt)

###########################################  Thermocouple Calib
# extend data set a bit for better extrapolation
t_data =      [50] + tmp + [210]
r_data = [18626.0] + rth + [0.0]
#t_data = tmp
#r_data = rth

#############################################
#### set up fitting weights so that error is smallest where we CARE most
w = np.zeros(len(t_data))
# fit tighest around these two points(!)
p1 = 94.5     #   fermentation
p2 = 185.0    #   denature

for i,t in enumerate(t_data):  # go through temperatures
    d1 = abs((t-p1)/float(p1))  # normalized distance to key temp
    d2 = abs((t-p2)/float(p2))
    pwr = 4
    d = max(0.2,(1-d1)**pwr,(1-d2)**pwr)
    w[i] = d
    #w[i] = max(0.2, (1-d*d))
    #print('{} {}, d1: {:4.2f} d2: {:4.2f} '.format(i,t,d1,d2,d))
 
#quit()
    

#ax,fig = plt.subplots() 
#a = plt.gca()
#a.set_ylim([0,1.5]) 
#plt.plot(t_data,w)
#plt.title('Polyfit weighting')



#######################################################################
#
# polynomial fits
#
##

#   fit temperature to resistance with 3rd order poly.
t2R_curve      = npp.Polynomial([1, 1, 1]).fit(t_data,r_data,3,w=w) # Resistance(T)

## gen some training data for r to temp fit.
#      ** assumes that t2R_curve is acceptable

npts = 60
tinit = 80
tfinal = 110
dt = float((tfinal-tinit))/float(npts)
trng_t = np.zeros(npts)
trng_r = np.zeros(npts)
tempr = tinit
for i in range(npts):
    trng_t[i] = tempr
    trng_r[i] = t2R_curve(tempr)
    tempr += dt

# get weights for the Resistance domain
rp0 = 10000
rp1 = 6400  # @ T = 95
rp2 = 50  # @ T = high value
wr = np.zeros(len(trng_r))
for i,r in enumerate(trng_r):    
    d0 = min(1.0,abs((r-rp0)/float(rp0)))  # normalized distance to key temp
    d1 = min(1.0,abs((r-rp1)/float(rp1)))  # normalized distance to key temp
    #d2 = min(1.0,abs((r-rp2)/float(rp2)))
    d0 = 1
    d2 = 1
    pwr = 4
    d = max(0.05 , (1-d0)**pwr,(1-d1)**pwr,(1-d2)**pwr)
    wr[i] = d
    
    
##  fit R2t based on "training data" which is richer but based on t2R fit(!)
R2t_curve = npp.Polynomial([1, 1, 1]).fit(trng_r,trng_t,2,w=wr) # Temperature(R)
#R2t_curve = npp.Polynomial([1, 1, 1]).fit(trng_r,trng_t,3) # Temperature(R)
print('polynomial fit, R2t:', R2t_curve)

# temp vars for plotting T(R) curve fit
tmpr = []
r2t_poly_fit = []
for r in range(0,18000,200):
    tmpr.append(r)
    r2t_poly_fit.append(R2t_curve(r))
    
    
    
print('poly example test: ')

R = 6500
print('R: {:} T(R): {:6.2f}'.format(R,R2t_curve(R)))
pcoef = np.array([ 154.3837628 ,  266.36830154 , -63.66957973, -377.5040949 ])
pwrs = np.array([ 1, R, R*R, R*R*R ])
pwrs = np.array([ R*R*R, R*R, R, 1 ])
print('R: {:} dotpoly: {:6.2f}'.format(R,np.dot(pcoef,pwrs)))

Ttmp = pcoef[3]*1 + pcoef[2]*R + pcoef[1]*R*R + pcoef[0]*R*R*R
print('R: {:} fpoly: {:6.2f}'.format(R,Ttmp))

# create ohms to temperature lookup table 
Npts = 256 # 8 bit index
dR = 25000/Npts  # Ohms
R0 = 0.0         # Ohms

Ohms2Temp = []   # tenths of a degree

ltx = []
lty = []
R = R0
for i in range(Npts):
    Ohms2Temp.append(int(R2t_curve(R)*10.0))
    lty.append(R)
    ltx.append(0.10*Ohms2Temp[-1])
    R += dR
    
#print('Lookup Table:',Ohms2Temp)
    
if True:
        
    ax,fig = plt.subplots() 
    plt.plot(t_data,r_data, 'r', r2t_poly_fit,tmpr,':g')
    plt.title('NTC Thermocouple resistance vs. temperature (deg F)')
    a = plt.gca()
    plt.grid()
    a.set_xlabel('Temperature (F)')


    ax,fig = plt.subplots() 
    plt.plot(tmpr, r2t_poly_fit,':g', r_data,t_data)
    plt.title('NTC Thermocouple resistance vs. temperature (deg F)')
    a = plt.gca()
    plt.grid()
    a.set_xlabel('Resistance')
    a.set_ylabel('Temperature')


##############################################      ADC Resolution
adcvals = []
if True:
    Vcc = 5.0
    R1 = 2000.0
    R1 = 6426.0  # R(T=95F)
    R1 = 5000.0  # best ADC resolution in Ferm range
    for r in rth:  # estimate digital values
        Vin = Vcc*r/(R1+r)
        adcvals.append(1023*Vin/Vcc)
    dadt = []
    for i in range(len(adcvals)-1):
        da = adcvals[i+1]-adcvals[i]
        dt = tmp[i+1]-tmp[i] # change in *temperature*
        dadt.append(da/dt)
    dadt.append(0)
    
    if False:
        ax,fig = plt.subplots() 
        plt.plot(tmp, adcvals,tmp,dadt)
        plt.title('ADC values vs. temperature')
        a = plt.gca()
        plt.grid()
        a.set_ylim([0,1030])
        a.set_xlabel('Temperature (F)')
        a.set_ylabel(' ADC value (0-1023)')

        ax,fig = plt.subplots() 
        plt.plot(tmp,dadt)
        plt.title('Delta ADC / Delta T vs T')
        a = plt.gca()
        plt.grid()
        a.set_ylim([-15,2])
        a.set_xlabel('Temperature (F)')
        a.set_ylabel('da/dt of ADC value (0-1023)')
    

tamb = 70.0
###########################################  Phase I
#
#   constant full power until temp = Tdenature
#

if False:
#######  Simulation

    #  Power input from electric heating element
    Uexp = np.zeros(len(Texp))
    for i,u in enumerate(Uexp):
        if Texp[i] <= 205:
            Uexp[i] = Pheater

    #  Open Loop response of plant with no controller 
    t, y1, x = ctl.forced_response(ss_mod, Texp, [ Uexp ],X0=[44.0-tamb]) 

    y1 = tempfix(y1,tamb)

    ax,fig = plt.subplots() 
    plt.plot(t,y1,ti,tmp) #,th,Rmod)  # 

    ptarget(plt, t[-1],185.00-tamb)

    
    a = plt.gca()
    a.set_ylim([0,350])
    plt.title('Phase I: constant max pwr')


plt.show()
