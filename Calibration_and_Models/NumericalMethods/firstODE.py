import numpy as np
import matplotlib.pyplot as plt
import control as ctl
import sys

tmax = 2.0
dt = 0.05
tr = np.arange(0,tmax, dt)
x = np.zeros(len(tr))

#xd = -ax

a = 1.0  # TC

x[0] = 10  # I.C.
for i,t in enumerate(tr):
    if i>0:
        xd = -a*x[i-1] # 1st order ODE
        x[i] = x[i-1] + xd*dt  # rectangular 

ax,fig = plt.subplots()
plt.plot(tr,x)
a = plt.gca()

plt.title('first order')
plt.show()
