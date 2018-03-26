import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

f = 2.5
numtaps = 100
LOAD_CELL_SAMPLE_FREQ = 80.0
b = signal.firwin(numtaps, f, window='hamming', nyq=LOAD_CELL_SAMPLE_FREQ/2.0)

w, h = signal.freqz(b)

fig = plt.figure()
plt.title('Digital filter frequency response')
ax1 = fig.add_subplot(111)

plt.plot(w, 20 * np.log10(abs(h)), 'b')
plt.ylabel('Amplitude [dB]', color='b')
plt.xlabel('Frequency [rad/sample]')

ax2 = ax1.twinx()
angles = np.unwrap(np.angle(h))
plt.plot(w, angles, 'g')
plt.ylabel('Angle (radians)', color='g')
plt.grid()
plt.axis('tight')
plt.show()