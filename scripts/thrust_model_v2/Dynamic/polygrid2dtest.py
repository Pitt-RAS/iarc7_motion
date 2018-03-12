from numpy.polynomial import Polynomial as P
import numpy as np

x = np.arange(5.0)
y = np.arange(5.0)

c = [[1., 1.], [0., 0.]]

print x
print y
print c

print np.polynomial.polynomial.polygrid2d(x, y, c)