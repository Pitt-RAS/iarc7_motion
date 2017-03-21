#!/usr/bin/env python2

from scipy.optimize import curve_fit
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

t1 = 14./13.
d1 = 0.27
t2 = 11./10.
d2 = 0.14

d0 = (d2 - d1) / np.log((t1-1) / (t2-1))
A_ge = (t1 - t2) / (np.exp(-d1/d0) - np.exp(-d2/d0))

def poly(x, a):
    '''
    Returns a[0]*x**n + a[1]*x**(n-1) + ... + a[n]
    '''
    if not a:
        return 0
    return x * poly(x, a[:-1]) + a[-1]

def make_surface_from_degrees(deg1, deg2):
    def f(x, *args):
        if len(args) != deg1 + deg2 + 1:
            raise TypeError, "Wrong number of arguments"
        return ((x[:,0]**deg1 + poly(x[:,0], args[:deg1]))
              * poly(x[:,1], args[deg1:])
              * (1 + A_ge * np.exp(-x[:,2] / d0)))
    return f

# DATA
#
# Each array here is one sweep, although some change direction multiple times
#
# Points are in the form (force, throttle, vbat, dist to ground)
#
# All points are taken with one motor attached to a prop and all others
# spinning freely unless otherwise noted

data = (
# this sweep doesn't line up with the others (you can uncomment and see for
# yourself, it's the line way above the fitted surface)
#     np.array((
#             (470,2011,12.5,1.5),
#             (435,1941,12,1.5),
#             (405,1881,12,1.5),
#             (375,1818,11.9,1.5),
#             (360,1753,11.9,1.5),
#             (350,1688,11.9,1.5),
#             (344,1654,11.9,1.5),
#             (325,1605,11.0,1.5),
#             (295,1557,11.0,1.5),
#             (277,1511,11.0,1.5),
#             (260,1485,11.0,1.5),
#         )),

    np.array((
            (15,1220,12.4,1.5),
            (27,1255,12.4,1.5),
            (42,1291,12.4,1.5),
            (57,1331,12.4,1.5),
            (82,1393,12.4,1.5),
            (105,1443,12.3,1.5),
            (121,1486,12.3,1.5),
            (135,1523,12.3,1.5),
            (150,1563,12.3,1.5),
            (166,1596,12.2,1.5),
            (176,1621,12.2,1.5),
            (160,1590,12.2,1.5),
            (141,1545,12.2,1.5),
            (130,1523,12.2,1.5),
            (113,1473,12.2,1.5),
            (91,1425,12.2,1.5),
            (76,1390,12.2,1.5),
            (64,1356,12.2,1.5),
            (50,1323,12.2,1.5),
            (35,1287,12.2,1.5),
            (24,1256,12.2,1.5),
        )),

    np.array((
            (42,1286,12.1,1.5),
            (101,1431,12,1.5),
            (156,1565,11.9,1.5),
            (208,1665,11.9,1.5),
            (245,1734,11.8,1.5),
            (297,1811,11.7,1.5),
            (330,1885,11.5,1.5),
            (375,1972,11.4,1.5),
            (380,2010,11.3,1.5),
        )),

    np.array((
            (390,2011,11.3,0.2),
            (300,1875,11.4,0.2),
            (242,1774,11.5,0.2),
            (186,1663,11.5,0.2),
            (139,1555,11.6,0.2),
            (97,1450,11.6,0.2),
            (59,1351,11.6,0.2),
            (28,1265,11.6,0.2),
        )),

    np.array((
            (400,2010,11.2,0.4),
            (318,1871,11.3,0.4),
            (255,1796,11.3,0.4),
            (210,1720,11.4,0.4),
            (170,1655,11.4,0.4),
            (140,1581,11.4,0.4),
            (116,1521,11.4,0.4),
            (85,1431,11.5,0.4),
            (37,1303,11.5,0.4),
        )),

    # single prop multiple sweep
    np.array((
            (31,1283,11.2,1.5),
            (60,1355,11.2,1.5),
            (95,1457,11.2,1.5),
            (145,1591,11.1,1.5),
            (200,1710,11,1.5),
            (260,1815,10.9,1.5),
            (297,1886,10.9,1.5),
            (335,1958,10.8,1.5),
            (360,2011,10.7,1.5),
            (300,1913,10.8,1.5),
            (260,1843,10.9,1.5),
            (214,1760,10.9,1.5),
            (177,1686,11,1.5),
            (144,1621,11,1.5),
            (85,1451,11.1,1.5),
            (40,1315,11.1,1.5),
            (80,1428,11.1,1.5),
            (122,1537,11.1,1.5),
            (143,1606,11,1.5),
            (185,1690,11,1.5),
            (234,1786,10.9,1.5),
            (295,1884,10.8,1.5),
            (340,1973,10.7,1.5),
            (350,2011,10.7,1.5),
            (215,1770,10.9,1.5),
            (150,1642,10.9,1.5),
            (91,1475,10.9,1.5),
            (210,1752,10.9,1.5),
            (290,1893,10.8,1.5),
            (350,2011,10.6,1.5),
            (94,1494,10.9,1.5),
            (15,1248,11,1.5),
            (350,2010,10.6,1.5),
            # ^ originally dropped from 370 to 350
            (250,1835,10.5,1.5)
        )),

    # With second motor loaded with a prop
    np.array((
            (70,1371,11.2,1.5),
            (115,1508,11.1,1.5),
            (180,1662,11.1,1.5),
            (225,1755,11,1.5),
            (255,1818,10.9,1.5),
            (285,1871,10.9,1.5),
            (295,1910,10.8,1.5),
            (335,2010,10.7,1.5),
            # ^ originally dropped from 350 to 335
            (245,1837,10.8,1.5),
            (165,1688,10.8,1.5),
            (95,1491,10.9,1.5),
            (180,1688,10.9,1.5),
            (250,1846,10.7,1.5),
            (283,1906,10.5,1.5),
            (320,2011,10.1,1.5)
        ))
    )

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
    deg_voltage = 3
    f = make_surface_from_degrees(deg_command, deg_voltage)

    fit = curve_fit(f,
                    data[:,1:],
                    data[:,0],
                    p0=tuple(1 for _ in range(deg_command + deg_voltage + 1)),
                    maxfev=10000)

    print 'Fit:'
    print fit[0]
    print 'Uncertainty:'
    print fit[1]

    print 'Result:'
    print 'A_ge = {}'.format(A_ge)
    print 'd0 = {}'.format(d0)
    print 'P(V) = ({})V^3 + ({})V^2 + ({})V + ({})'.format(*fit[0][2:])
    print 'C_0 = T / (P(V) * (1 + ({A_ge})e^(-d / ({d0}))))'.format(A_ge=A_ge, d0=d0)
    print 'Command = 0.5 * (-({b}) + sqrt(({b})^2 - 4({c} - C_0)))'.format(b=fit[0][0], c=fit[0][1])

    samples = np.mgrid[min(data[:,1]):max(data[:,1]):50j,
                       min(data[:,2]):max(data[:,2]):50j]
    for gdist, color in ((0.2,'r'), (0.4,'g'), (1.5,'yellow')):
        axes.plot_surface(samples[0],
                          samples[1],
                          f(np.concatenate((
                                    samples,
                                    [gdist*np.ones_like(samples[0])]
                                )).reshape(3, -1).T,
                            *fit[0]).reshape(samples[0].shape),
                          color=color)
    plt.show()
