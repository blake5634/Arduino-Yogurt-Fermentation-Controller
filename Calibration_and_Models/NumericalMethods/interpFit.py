import numpy as np
import numpy.polynomial.polynomial as npp
import matplotlib.pyplot as plt
import control as ctl

def array2C(ar,name):
    nelem = len(ar)
    print('float {:}[] = '.format(name)+'{',end='')
    nline = 5
    l = 0
    np = 0
    for a in ar:
        print('{:9.4f}'.format(a),end =  '')
        np += 1
        if np < nelem:
            print(', ',end =  '')
        if np%nline == 0:
            print('') # newline
    print('};')
    print('')

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
# Slow Cooker Data
f = open('waterbath.csv','r')
# time, etmin, temp F, rNTC (Kohms)
S
ti = []
tmp = []
rth_bk = []
rth_wh = []
rth_avg = []

for l in f:
    d = l.split(',')
    tmp.append(float(d[2])) # avg ref temp in deg F
    rth_bk.append(float(d[3]))  # r in ohms (black Thermocouple)(from KOhm)
    rth_wh.append(float(d[4]))  # r in ohms (white Thermocouple)( KOhm)
    rth_avg.append(0.5*(float(d[3])+float(d[4])))
    
    
## Put a point out beyond hightest temp so can interp.
tmp.append(225.0)
rth_bk.append(115.0)
rth_wh.append(115.0)
rth_avg.append(115.0)

rth_bk.sort(reverse=True)
rth_wh.sort(reverse=True)
rth_avg.sort(reverse=True)
tmp.sort(reverse=False) # because NTC


npts = 75
r0 = 1400
r2 = 25001
dr = float(r2-r0)/float(npts)
rtest = []
ttest_bk = []
ttest_wh = []
ttest_avg = []
err_wh = []
print ('len(rth) = ',len(tmp))
for i in range(npts):
    rtest.append(r0)
    ttest_bk.append(interp(r0,rth_bk,tmp))
    ttest_wh.append(interp(r0,rth_wh,tmp))
    ttest_avg.append(interp(r0,rth_avg,tmp))
    tmp_estimate = 0.5*(ttest_bk[-1]+ttest_wh[-1]) # avg of two curves
    err_wh.append(abs(ttest_avg[-1]-ttest_bk[-1]))
    print('error:',rtest[i],err_wh[-1])
    r0 += dr

emax = 0.0
emaxpct = 0.0
t_emax = 0.0
for i,e in enumerate(err_wh):
    ttrue = 0.5*(ttest_bk[i]+ttest_wh[i])
    if (rtest[i] < 12000) and (rtest[i] > 3000):
        if e > emax:
            print ('----',ttrue, e)
            emax = e
            emaxpct = e/ttest_wh[i]
            t_emax = ttrue

print('Max in-range error (avg) = ',emax,'deg F', emaxpct*100.0,'%')
print('     at t = ',t_emax)

#############   Generate some C code

array2C(rth_wh,'rth_white')
array2C(rth_bk,'rth_black')
array2C(tmp, 'temps')


ax,fig = plt.subplots() 
plt.plot(rth_bk ,  tmp, '.r',
         rth_wh,   tmp, '.r',
         rth_avg,  tmp, '.r',
         rtest,ttest_avg,':g',
         rtest,ttest_avg,'.y', 
         rtest,ttest_bk,':g',
         rtest,ttest_bk,'.k', 
         rtest,ttest_wh,':g',
         rtest,ttest_wh,'.b')
plt.title('NTC Thermocouple resistance vs. temperature (deg F)')
a = plt.gca()
plt.grid()
a.set_xlabel('Resistance')
a.set_ylabel('Temperature')

plt.show()
