#!/usr/bin/env python2

from scipy.optimize import curve_fit
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import os.path

'''
Thrust model data and curve fitting script
'''

def poly(x, a):
    '''
    Returns a[0]*x**n + a[1]*x**(n-1) + ... + a[n]
    '''
    if not a:
        return 0
    return x * poly(x, a[:-1]) + a[-1]

def make_surface_from_degrees(deg1, deg2):
    def f(x, *args):
        A_ge = args[-2]
        d0 = args[-1]
        args = args[:-2]
        if len(args) != deg1 + deg2 + 1:
            raise TypeError, "Wrong number of arguments"
        return ((x[:,0]**deg1 + poly(x[:,0], args[:deg1]))
              * poly(x[:,1], args[deg1:])
              * (1 + A_ge * np.exp(-x[:,2] / d0)))
    return f

# DATA
#
# Each array here is one sweep
#
# Points are in the form (force, throttle, vbat, dist to ground)
data_dir = 'thrust_testing_data2'
data = []
for filename in os.listdir(data_dir):
    data.append(np.loadtxt(os.path.join(data_dir, filename),
                           delimiter=','))
    data[-1][:,(0, 1)] = data[-1][:,(1, 0)]
    if data_dir == 'Thrust_Testing_Data':
        data[-1] = np.concatenate((data[-1], 1.5*np.ones((data[-1].shape[0], 1), dtype=np.float)), axis=1)
        data[-1][:,0] -= 3.13e5
        data[-1][:,0] /= 397756
data = tuple(data)

if __name__ == '__main__':
    fig = plt.figure()
    axes = fig.add_subplot(111, projection='3d')
    for line_index in range(len(data)):
        axes.plot(data[line_index][:,1],
                  data[line_index][:,2],
                  data[line_index][:,0],
                  color=plt.get_cmap('gnuplot')(line_index / float(len(data))))

    data = np.concatenate(data)

    # Add csv files here to be displayed in bold
    extra_files = tuple()
    for filename in extra_files:
        data2 = np.loadtxt(filename, delimiter=',')
        data2[:,(0,1)] = data2[:,(1,0)]
        axes.plot(data2[:,1], data2[:,2], data2[:,0], 'red', linewidth=5)

    deg_command = 2
    deg_voltage = 1
    f = make_surface_from_degrees(deg_command, deg_voltage)

    fit = curve_fit(f,
                    data[:,1:],
                    data[:,0],
                    p0=tuple(1 for _ in range(deg_command + deg_voltage + 1)) + (0.13, 0.5),
                    maxfev=10000)

    print 'Fit:'
    print fit[0]
    print 'Uncertainty:'
    print fit[1]

    A_ge = fit[0][-2]
    d0 = fit[0][-1]

    print 'Result:'
    print 'A_ge = {}'.format(A_ge)
    print 'd0 = {}'.format(d0)
    print 'P(V) = ' + ' + '.join('({})V^{}'.format(fit[0][2+i], i) for i in range(deg_voltage, -1, -1))
    print 'C_0 = T / (P(V) * (1 + ({A_ge})e^(-d / ({d0}))))'.format(A_ge=A_ge, d0=d0)
    print 'Command = 0.5 * (-({b}) + sqrt(({b})^2 - 4({c} - C_0)))'.format(b=fit[0][0], c=fit[0][1])

    samples = np.mgrid[min(data[:,1]):max(data[:,1]):50j,
                       min(data[:,2]):max(data[:,2]):50j]
    for gdist, color in ((0.3, 'r'), (0.7, 'g'), (1.5,'yellow'), (2.5, 'b')):
        axes.plot_surface(samples[0],
                          samples[1],
                          f(np.concatenate((
                                    samples,
                                    [gdist*np.ones_like(samples[0])]
                                )).reshape(3, -1).T,
                            *fit[0]).reshape(samples[0].shape),
                          color=color)
    plt.show()
