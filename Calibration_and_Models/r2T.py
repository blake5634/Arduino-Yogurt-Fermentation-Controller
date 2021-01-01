import numpy as np
import numpy.polynomial.polynomial as npp
import matplotlib.pyplot as plt
import control as ctl
import sys


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


def interp(rm,R,T):
    tval = -1
    for i in range(len(R)):
        #if R[i] < 3000 and R[i] > 2000:
            #print ('interp:',rm,R[i])
        if rm >= R[i]:
            dTdR = float(T[i]-T[i-1])/float(R[i]-R[i-1])
            tval = float(T[i]) + float((rm-R[i])) * dTdR
            break
        else:
            pass 
    #if rm < 3000 and rm > 2000:
        #print('rm {:8.1f} R[i]: {:8.1f} T[i]: {:4.1f} dTdR: {:.3f} tval: {}'.format(rm,R[i],T[i],dTdR,tval))
    return tval
        
##########################################   Read in data
# Slow Cooker Data
f = open('milkdata.csv','r')
# time, etmin, temp F, rNTC (Kohms)

if len(sys.argv)!= 2:
    print('Usage: r2T.py Rvalue')
    quit()
    
ti = []
tmp = []
rth = []

for l in f:
    d = l.split(',')
    ti.append(int(d[1])) # elapsed min
    tmp.append(float(d[2])) # temp in deg F
    rth.append(1000.0*float(d[3]))  # r in ohms (from KOhm)
    
rth.sort(reverse=True)
tmp.sort(reverse=False) # because NTC

R = float(sys.argv[1])
print(R, 'T = {:4.2f} degF'.format(interp(R,rth,tmp)))
