import math
import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
import scipy.fftpack as fftpack
from scipy import signal
from scipy.optimize import lsq_linear
from mpl_toolkits.mplot3d import Axes3D
from numpy.polynomial import Polynomial as P
import yaml
import datetime

# Derived from https://wiki.bitcraze.io/misc:investigations:thrust
def voltage_to_thrust(voltage):
    return (3.747*voltage**2 + 5.804*voltage + 0.745)/1000.0

def create_voltage_to_thrust(voltages, thrusts):
    z = np.polyfit(voltages, thrusts, deg=poly_fit_degree)
    voltage_to_thrust = np.poly1d(z)
    fit_voltages = np.linspace(0, np.max(voltages)*1.2, 100)

    z = np.polyfit(thrusts, voltages, deg=poly_fit_degree)
    thrust_to_voltage = np.poly1d(z)
    fit_thrusts = np.linspace(0, np.max(thrusts)*1.2, 100)

    plt.figure()
    plt.subplot(211)
    plt.plot(voltages, thrusts, 'bo', fit_voltages, voltage_to_thrust(fit_voltages), 'r-')

    plt.subplot(212)
    plt.plot(thrusts, voltages, 'bo', fit_thrusts, thrust_to_voltage(fit_thrusts), 'r-')

    return voltage_to_thrust, thrust_to_voltage


def generate_2d_polynomial_terms(m_terms, m_degree, n_terms, n_degree):
    rows = len(m_terms)
    if(rows != len(n_terms)):
        raise ValueError('M terms and N terms do not have the same number of data points')

    terms = np.empty(rows*(m_degree+1)*(n_degree+1)).reshape(rows, (m_degree+1)*(n_degree+1))

    for row in range(0, rows):
        for i in range(0, m_degree+1):
            for j in range(0, n_degree+1):
                terms[row, (i*(n_degree+1))+j] = m_terms[row]**i * n_terms[row]**j

    return terms

thrust_points_count = 10
voltage_points_count = 10
Ts = 0.020
def generate_model_map(voltage_to_thrust):

    thrust_min = min_thrust
    thrust_max = max_thrust
    voltage_min = min_voltage
    voltage_max = max_voltage

    thrust_points = np.linspace(thrust_min, thrust_max, thrust_points_count, endpoint=True)

    voltage_points = np.linspace(voltage_min, voltage_max, voltage_points_count, endpoint=True)

    voltage_points_mesh, thrust_points_mesh  = np.meshgrid(voltage_points, thrust_points)
    thrust_points_mesh = thrust_points_mesh.flatten()
    voltage_points_mesh = voltage_points_mesh.flatten()

    #t_p = [0.7111111111111111]
    #v_p = [9.333333333333332]

    #print time_constant_function(t_p, v_p)

    T = voltage_to_thrust(voltage_points_mesh)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.scatter(thrust_points_mesh, T, voltage_points_mesh, alpha=1.0)

    a = np.array(zip(voltage_points_mesh, T)).reshape(thrust_points_count, voltage_points_count, 2).tolist()
    #print a
    voltage_to_jerk_mapping = zip(thrust_points.tolist(), a)
    #print voltage_to_jerk_mapping
    #print voltage_to_jerk_mapping.shape

    voltage_to_jerk_model = {'thrust_min': thrust_min,
                             'thrust_max': thrust_max,
                             'voltage_min': voltage_min,
                             'voltage_max': voltage_max,
                             'timestep': Ts,
                             'mapping': voltage_to_jerk_mapping}

    ax.scatter(thrust_points_mesh, thrust_points_mesh, voltage_points_mesh, alpha=0.3)

    ax.plot_trisurf(thrust_points_mesh, voltage_to_thrust(voltage_points_mesh), voltage_points_mesh, alpha=0.3)

    ax.set_xlabel('Current Thrust (kg)')
    ax.set_ylabel('Next Thrust (kg)')
    ax.set_zlabel('Applied Voltage (V)')

    return voltage_to_jerk_model

min_voltage = 0
max_voltage = 3.2

min_thrust = 0
max_thrust = voltage_to_thrust(max_voltage)

response_lag = 0.0

poly_fit_degree = 2

voltages = np.linspace(min_voltage, max_voltage, 20)
thrusts = [voltage_to_thrust(v) for v in voltages]
voltage_to_thrust_poly, thrust_to_voltage_poly = create_voltage_to_thrust(voltages, thrusts)

voltage_to_jerk_model = generate_model_map(voltage_to_thrust)

output_model = {'response_lag': response_lag,
                'small_thrust_epsilon': (max_thrust-min_thrust)/200,
                'thrust_to_voltage': thrust_to_voltage_poly.coef.tolist(),
                'voltage_to_jerk': voltage_to_jerk_model}

with open('thrust_model_crazyflie.yaml', 'w') as f:
    f.write('# Generated on {}\n\n'.format(str(datetime.datetime.now())))
    f.write(yaml.safe_dump(output_model))
    f.write('\n')

plt.show()
