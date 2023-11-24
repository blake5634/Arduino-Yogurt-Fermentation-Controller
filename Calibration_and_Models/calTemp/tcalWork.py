import numpy as np
import numpy.polynomial.polynomial as npp
import matplotlib.pyplot as plt
import control as ctl

WHITESENSOR    = 0
BLACKSENSOR    = 1

SENSOR_OFFSET_WHITE              =  2.0
SENSOR_CORRECTION_DENATURE_WHITE =  -4.0
SENSOR_CORRECTION_AMBIENT_WHITE  =  0.0
SENSOR_CORRECTION_NULL_WHITE     =  160
Tferment = 110.0
Tdenature = 195.0

def readcsv(name):
    f = open(name,'r')
    #
    #
    #   Read in the .csv data
    #
    tref = []   # actual temp
    tcalc = []  # old interp result
    rd = []    # resistance data
    next(f)
    next(f)  # burn blank and header lines

    for l in f:
        d = l.split(',')
        tref.append(float(d[0]))
        tcalc.append(float(d[1]))
        rd.append(float(d[2]))
    print('I got {:} data points'.format(len(rd)))
    return tref,tcalc,rd

def F2K(f):
    K = (f-32)/1.8 + 273.15
    return K
def K2F(k):
    F = (k-273.15)*9/5 + 32
    return F

def makeSHeqn(rd,tref):  # actually defer
    rlog = np.zeros(len(rd))
    for i,r in enumerate(rd):
        rlog[i] = np.ln(r)
    ## not done: see https://www.mstarlabs.com/sensors/thermistor-calibration.html



def R2T(r, sensor):
    if sensor == WHITESENSOR:
        #    update with May'23 data (average close points)
        #         last 2 pts are "fake" to continue interploation
        p = [29400,13000,10920,7565,5080,4680,2400,2080,1530,1105, 1100]
        tarray = [32,65,74,93,113,118,157,166,184, 207, 213]
    else:
        str = " Error: R2T()"

    nintpts = len(tarray)
    tval = -1.0

    minr = 1100.0 #fake
    maxr = 29400.0 #np.max(rth_white)
    minT = 32.0 # np.min(tarray)
    maxT = 213.0  # fake

    # note NTC
    if r > maxr:
        return minT
    if r < minr:
        return maxT

    # now interpolate
    for i in range(nintpts):
        #print('..> r: {:} i: {:} p[i+1]: {:}'.format(r,i,p[i+1]))
        if  r >= p[i]:
            dTdR = (tarray[i] - tarray[i-1]) / (p[i] - p[i-1])
            tval = tarray[i] + (r - p[i]) * dTdR
            break

    return float(tval)

def interp(rm,R,T):
    tval = -1
    for i in range(len(R)):
        #if R[i] < 3000 and R[i] > 2000:
            #print ('interp:',rm,R[i])
        if rm >= R[i]:
            dTdR = float(T[i]-T[i-1])/float(R[i]-R[i-1]) # i-1 because reverse order
            tval = float(T[i]) + float((rm-R[i])) * dTdR
            break
        else:
            pass 
    #if rm < 3000 and rm > 2000:
        #print('rm {:8.1f} R[i]: {:8.1f} T[i]: {:4.1f} dTdR: {:.3f} tval: {}'.format(rm,R[i],T[i],dTdR,tval))
    return tval
        
##########################################   Read in data
# Calibration data
#    ref: Thermapen digital cooking Thermo
#
fname = 'calData10-May-23_WHITE.csv'
#
#   Tref,  Tcalc, R
#      Tref = Thermapen
#      Tcalc = uncorrected interopolation from 2021
#      R = displayed R from
'''
float readResistance(){
    float Vcc = 5.0;
    float R1 = 5000.0;
    float Vin = (5.0/1023.0) * float(ain());
    float Rth = R1 / ((Vcc/Vin)-1.0);
    return Rth  ;
}
'''

tref,tcalc,r = readcsv(fname)

def r2Tv3(r):
    # linear term
    p1x = 110   # from orig data pts
    p1y = 207
    p2x = 2940
    p2y = 32
    a  = (p2y-p1y)/(p2x-p1x)
    b  = p1y - a*p1x

    # parabolic term
    pp1 = -55  # deg F
    r2 = (p2x-p1x)/2.0
    r0 = p1x + r2
    pterm = pp1*(1.0 - ((r-r0)/r2)**2)
    return a*r + b + pterm


#  add some fake data points to help polynomial fitting
#    (these come from reading the plot of r vs T(!)

#r.append(2000)
#tref.append(50)
#r.append(2500)
#tref.append(42)

# "X" is resistance r
# "Y" is ref temperature
# deg is degree

#deg = 4
#pc, info = npp.polyfit(r, tref, deg,full=True)

#print('polyfit:')
#print('coeffs', pc)
#print('info: ')
#print('residuals', info[0])
#print('rank',info[1])
#print('singvals',info[2])
#print('rcond',info[3])

rp = np.arange(500,30000,500) # resistance values
#tnew = npp.polyval(rp, pc)
tnew = []
rnew = []
for i,r1 in enumerate(rp):
    tt = R2T(r1,WHITESENSOR)
    rnew.append(r1)
    print('r: {:8.2f}  temp: {:6.2f}'.format(r1,tt))
    tnew.append(tt)
#print('rp:',rp)
#print('tnew',tnew)
print('lens:', len(r),len(tnew))

#rth_bk.sort(reverse=True)
#rth_wh.sort(reverse=True)
#rth_avg.sort(reverse=True)
#tmp.sort(reverse=False) # because NTC

if False:
    ax,fig = plt.subplots()
    plt.plot(tref,tcalc,tref,tref)
    plt.title('original interp vs. Accurate Temp. (deg F)')
    a = plt.gca()
    plt.grid()
    a.set_ylabel('Orig interp value')
    a.set_xlabel('Accurate Temperature')


    ax,fig = plt.subplots()
    plt.plot(r,tcalc)
    plt.title('original interp vs. R (Ohm)')
    a = plt.gca()
    plt.grid()
    a.set_ylabel('Original Interp')
    a.set_xlabel('R (Ohm)')


ax,fig = plt.subplots()
plt.plot(r,tref,rnew,tnew)
plt.title('Accurate Temp vs. R with Calib: Tdenature')
a = plt.gca()
plt.grid()
a.set_xlim([1000,3000])
a.set_ylim([150,210])
a.set_ylabel('Temp (deg F)')
a.set_xlabel('R (Ohm)')


ax,fig = plt.subplots()
plt.plot(r,tref,rnew,tnew)
plt.title('Accurate Temp vs. R with Calib: Tferment')
a = plt.gca()
plt.grid()
a.set_xlim([5200,5700])
a.set_ylim([105,115])
a.set_ylabel('Temp (deg F)')
a.set_xlabel('R (Ohm)')


ax,fig = plt.subplots()
plt.plot(r,tref,rnew,tnew)
plt.title('Accurate Temp vs. R with Calib: T Fridge')
a = plt.gca()
plt.grid()
a.set_xlim([19000, 23000])
a.set_ylim([45,55])
a.set_ylabel('Temp (deg F)')
a.set_xlabel('R (Ohm)')

plt.show()
