import pylab
import sys

def readdata(filename):
    fid = open(filename,'r')
    data = []
    for line in fid:
        line = line.split()
        if len(line) < 2:
            continue
        else:
            data.append([float(line[0]), float(line[1])])
    fid.close()
    data = pylab.array(data)
    return data

for file in sys.argv[1:]:
    data = readdata(file)
    t = data[:,0]
    temp = data[:,1]
    pylab.plot(t,temp,'.')
    pylab.grid('on')
    pylab.xlabel('time (sec)')
    pylab.ylabel('temp (C)')
pylab.show()
