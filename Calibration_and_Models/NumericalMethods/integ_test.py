import numpy as np
import matplotlib.pyplot as plt
import control as ctl
import sys

 
tamb = 0

# Default Parameters
Params = {}
Params['AntiWindup'] = True
Params['Emax'] = 25 # deg F
Params['Pmax'] = 300
Params['Tdenature'] = 190
Params['Tferment']  =  95 
Params['Tamb'] = tamb
Params['Tstart'] = tamb
Params['dt'] = .05 #simulation Dt, minutes
Params['Ctldt'] = 1.0  # control sample time
Params['Ca'] = -(1.0/310.0+1.0/2260.0)
Params['Cb'] = 1.0/310.0

Params['integrator'] = 'Euler'
#Params['integrator'] = 'Rectangular'  #untested

Params['Debug'] = False

pi = 3.1415926
dt = 0.25   #radians

t = np.arange(0,4*pi,dt)

y = np.zeros(len(t))
yEint = y.copy()
yRint = y.copy()
yref = y.copy()

for i,tp in enumerate(t):
    y[i] = np.sin(tp)
    yref[i] = -1*np.cos(tp)
    
## Perform numerical integration to get cos(t)

# intial conditions
yRint[0] = -1.0
yEint[0] = -1.0
for i,tp in enumerate(t):
    x = y[i]
    if i>0:
        dy = y[i] - y[i-1]
        dydt = dy/dt
        # Rectangular
        yRint[i] = yRint[i-1] + dt*y[i]
        # Euler
        yEint[i] = yEint[i-1] + 0.5*dt*(y[i]+y[i-1])


###########   Plotting


ax,fig = plt.subplots()
plt.plot(t,y,'g',t,yRint,'b',t,yEint,'r',t,yref,'.g') 
#a = plt.gca() 
#a.set_ylim([60,120])
#a.set_xlim([0,100]) 
plt.title('Integration Test')


plt.show()
