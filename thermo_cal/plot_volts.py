import pylab

data = pylab.loadtxt('thermo_volts.txt')
T = data[:,0]
V = 1.0e-3*data[:,1]
fit = pylab.polyfit(V,T,1)
V_fit = pylab.linspace(V.min(),V.max(),500)
T_fit = pylab.polyval(fit,V_fit)
print 'fit poly: T*%f + %f'%(fit[0], fit[1])
pylab.plot(V,T,'bo')
pylab.plot(V_fit, T_fit, 'r')
pylab.xlabel('(V)')
pylab.ylabel('(T)')
pylab.title('thermocouple calibration -- data sheet')
pylab.show()
