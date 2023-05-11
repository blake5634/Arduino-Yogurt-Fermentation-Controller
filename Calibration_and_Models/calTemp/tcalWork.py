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

def R2T(r, sensor):
    rth_white = [25000.0000, 19460.0000, 14450.0000, 11470.0000, 10480.0000,
                 7780.0000, 5340.0000, 3840.0000, 2290.0000, 1410.0000, 115.0000]

    rth_black = [30000.0000, 21370.0000, 15250.0000, 11700.0000, 10440.0000,
                 7420.0000, 4760.0000, 3240.0000, 1740.0000,  953.0000, 115.0000]

    tarray = [33.1000, 45.8000, 59.2000, 70.6000, 73.7000,
              88.4000, 107.7000, 125.8000, 156.8000, 194.6000, 225.0000]

    p = None
    nintpts = len(tarray)

    if sensor == WHITESENSOR:
        p = rth_white
    elif sensor == BLACKSENSOR:
        p = rth_black

    tval = -1.0

    for i in range(nintpts):
        if r > p[i+1]:
            dTdR = (tarray[i] - tarray[i - 1]) / (p[i+1] - p[i])
            tval = tarray[i] + (r - p[i]) * dTdR
            break

    if False and sensor == WHITESENSOR:
        retval = tval + SENSOR_OFFSET_WHITE
        hack_highT = 0.0
        if retval > Tferment:
            delta = SENSOR_CORRECTION_DENATURE_WHITE / (Tdenature - Tferment)
            hack_highT = (tval - Tferment) * delta
        retval += hack_highT
        return retval
    else:
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
f = open('calData10-May-23_WHITE.csv','r')
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

tref = []
tcalc = []
tnew = []
r = []

next(f)
next(f)  # burn blank and header lines


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

#
#
#   Read in the .csv data
#
#
for l in f:
    d = l.split(',')
    tref.append(float(d[0]))
    tcalc.append(float(d[1]))
    r.append(float(d[2]))
    
print('I got {:} data points'.format(len(r)))

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

#rp = np.arange(100,3000,100) # resistance values
#tnew = npp.polyval(rp, pc)
tnew = np.zeros(len(r))

for i,r1 in enumerate(r):
    tt = R2T(r1,WHITESENSOR)
    print('r: {:}  temp: {:}'.format(r1,tt))
    tnew[i] = tt
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
plt.plot(r,tref,r,tnew)
plt.title('Accurate Temp vs. R with Calib')
a = plt.gca()
plt.grid()
a.set_xlim([1000,6000])
a.set_ylim([100,210])
a.set_ylabel('Temp (deg F)')
a.set_xlabel('R (Ohm)')

plt.show()
