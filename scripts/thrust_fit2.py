#!/usr/bin/env python2

from scipy.optimize import curve_fit
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import os.path

'''
Thrust model data and curve fitting script

Ground effect seems to be consistent across all prop speeds, so it is accounted
for outside of the curve fitting process.  Ground effect model is as follows:

    $$ T = T_0 (A e^{-data/d_0} + 1) $$

$T_0$ is the thrust without ground effect, d_0 is the characteristic distance
for the effect, and A is the strength of the effect.  Measured results give
$T/T_0 = 14/13$ at $data = 0.27$ and $T/T_0 = 11/10$ at $data = 0.14$.  Solving the
system gives

    $$ d_0 = \\frac{d_2 - d_1}{\\log \\frac{t_1 - 1}{t_2 - 1}} $$

and

    $$ A = \\frac{t_1 - t_2}{e^{-d_1/d_0} - e^{-d_2/d_0}}, $$

where $d_2$ and $d_1$ are the respective measured distances and $t_1$ and $t_2$
are the respective ratios of $T$ to $T_0$.
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
