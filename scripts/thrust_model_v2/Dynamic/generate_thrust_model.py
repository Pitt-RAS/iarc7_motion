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


def remove_consecutive_duplicates(times, values):
    unique_times = []
    unique_values = []

    t_last_timestamp = -10
    for i in range(0, len(times)):
        if t_last_timestamp != times[i]:
            unique_times.append(times[i])
            unique_values.append(values[i])
            t_last_timestamp = times[i]
    return (unique_times, unique_values)

class Ramp(object):
    def __init__(self, start_throttle, end_throttle, start_data, end_data, settings):
        self.start_throttle = start_throttle
        self.end_throttle = end_throttle
        self.pre_transition_data = start_data
        self.post_transition_data = end_data

        self.TIME_CONSTANT_CALCULATION_RESOLUTION = settings['resolution']
        self.LOW_PASS_CUTOFF = settings['low_pass_cutoff']
        self.LOW_PASS_TAPS = settings['low_pass_taps']
        self.LOAD_CELL_SAMPLE_FREQ = settings['load_cell_sample_rate']

        # Try to find a file format, if no string assume 1.0
        try:
            self.FILE_FORMAT = settings['file_format']
            # Only other format is 0.1 right now
            self.THRUST_COLUMN = 0
            self.THRUST_TIME_COLUMN = 1
            self.VOLTAGE_COLUMN = 3
            self.VOLTAGE_STAMP_COLUMN = 5
            self.THROTTLE_COLUMN = 6
            self.THROTTLE_TIME_COLUMN = 7
        except KeyError as e:
            # Assume format is 1.0
            self.THRUST_COLUMN = 0
            self.THRUST_TIME_COLUMN = 1
            self.VOLTAGE_COLUMN = 4
            self.VOLTAGE_STAMP_COLUMN = 5
            self.THROTTLE_COLUMN = 7
            self.THROTTLE_TIME_COLUMN = 8
            

        # Use voltage/current measurements to find the start time since it is the fastest updated
        self._start_time = self.pre_transition_data[0][self.THROTTLE_TIME_COLUMN]/1000000.0
        # Use the ESC timestamp to find the actual transition start time
        # this is the time a new throttle value was sent to the ESC

        for i in range(0, len(self.post_transition_data)):
            if self.post_transition_data[i][self.THROTTLE_COLUMN] != self.start_throttle:
                self.transition_start_time = (self.post_transition_data[i][self.THROTTLE_TIME_COLUMN]/1000000.0) - self._start_time
                break

        data_concatenated = self.pre_transition_data + self.post_transition_data

        (self._original_thrust_times , self._original_thrusts) = remove_consecutive_duplicates([x[self.THRUST_TIME_COLUMN]/1000000.0 for x in data_concatenated], [x[self.THRUST_COLUMN] for x in data_concatenated])
        self._original_thrust_times = [x - self._start_time for x in self._original_thrust_times]

        (self._original_throttle_times, self._original_throttles) = remove_consecutive_duplicates([x[self.THROTTLE_TIME_COLUMN]/1000000.0 for x in data_concatenated], [x[self.THROTTLE_COLUMN] for x in data_concatenated])
        self._original_throttle_times = [x - self._start_time for x in self._original_throttle_times]

        (self._original_voltage_times , self._original_battery_voltages) = [x[self.VOLTAGE_STAMP_COLUMN]/1000000.0 for x in data_concatenated], [x[self.VOLTAGE_COLUMN] for x in data_concatenated]
        self._original_voltage_times = [x - self._start_time for x in self._original_voltage_times]
        self.throttle_interp = interp1d(self._original_throttle_times, self._original_throttles, kind='zero', bounds_error=False, fill_value=(self._original_throttles[0], self._original_throttles[-1]))
        self.motor_voltages = np.array([battery_voltage*self.throttle_interp(time)/100.0 for (time, battery_voltage) in zip(self._original_voltage_times, self._original_battery_voltages)])

        self.start_voltage = np.median([x for (t,x) in zip(self._original_voltage_times, self.motor_voltages) if t < self.transition_start_time])
        self.end_voltage = np.median([x for (t,x) in zip(self._original_voltage_times, self.motor_voltages) if t >= self.transition_start_time])

        self.start_thrust = np.median([x[self.THRUST_COLUMN] for x in self.pre_transition_data])
        self.end_thrust = np.median([x[self.THRUST_COLUMN] for x in self.post_transition_data])

        b = signal.firwin(self.LOW_PASS_TAPS, self.LOW_PASS_CUTOFF, window='hamming', nyq=self.LOAD_CELL_SAMPLE_FREQ/2.0)

        self._filtered_thrusts = signal.filtfilt(b, [1.0], self._original_thrusts)
        self._filtered_thrusts_interpolator = interp1d(self._original_thrust_times, self._original_thrusts, kind='linear')

        self.calculate_time_constant()
        self.fitted_response_curve_times = np.linspace(self.fitted_transition_start_time, self.rise_end_time+0.5, num=200, endpoint=True)
        self.fitted_response_thrust = (self.end_thrust-self.start_thrust)*(1-np.exp(-(self.fitted_response_curve_times-self.fitted_transition_start_time)/self.time_constant)) + self.start_thrust

    def __str__(self):
        return 'Ramp Object {} to {}'.format(self.start_throttle, self.end_throttle)

    def get_thrusts_and_times(self):
        RESAMPLE_FREQUENCY = 2000
        thrust_interpolator = interp1d(self._original_thrust_times, self._original_thrusts, kind='linear')
        num_samples = (self._original_thrust_times[-1]-self._original_thrust_times[0]) * RESAMPLE_FREQUENCY
        interpolated_times = np.linspace(self._original_thrust_times[0], self._original_thrust_times[-1], num=num_samples, endpoint=True)
        return (interpolated_times, thrust_interpolator(interpolated_times))
    
    def get_throttles_and_times(self):
        return (self._original_throttle_times, self._original_throttles)

    def get_start_thrust(self):
        return self.start_thrust

    def get_end_thrust(self):
        return self.end_thrust

    def get_fft(self):
        # Assume scale data came at 80 Hz even though there is slight variation
        weights = fftpack.fftshift(fftpack.fft(self._original_thrusts))
        power = np.abs(weights)**2
        frequencies = fftpack.fftshift(fftpack.fftfreq(power.size, d=1.0/self.LOAD_CELL_SAMPLE_FREQ))
        return (frequencies, power)

    def get_filtered_thrust(self):
        return (self._original_thrust_times, self._filtered_thrusts)        

    def calculate_time_constant(self):
        # The below codes works for both high->low and low->high transitions
        # Find 10% and 90% thresholds
        start_threshold = 0.1 * (self.end_thrust-self.start_thrust) + self.start_thrust
        end_threshold = 0.9 * (self.end_thrust-self.start_thrust) + self.start_thrust

        max_samples = int((self._original_throttle_times[-1] - self.transition_start_time) / self.TIME_CONSTANT_CALCULATION_RESOLUTION)

        # Find the time for the 10% threshold
        for i in range(0, max_samples):
            sample_time = self.transition_start_time + i * self.TIME_CONSTANT_CALCULATION_RESOLUTION
            thrust = self._filtered_thrusts_interpolator(sample_time)
            if start_threshold < end_threshold:
                if thrust > start_threshold:
                    self.rise_start_time = sample_time
                    self.rise_start_thrust = thrust
                    break
            else:
                if thrust < start_threshold:
                    self.rise_start_time = sample_time
                    self.rise_start_thrust = thrust
                    break

        # Find the time for the 90% threshold
        for i in range(0, max_samples):
            sample_time = self.transition_start_time + i * self.TIME_CONSTANT_CALCULATION_RESOLUTION
            thrust = self._filtered_thrusts_interpolator(sample_time)
            if start_threshold < end_threshold:
                if thrust > end_threshold:
                    self.rise_end_time = sample_time
                    self.rise_end_thrust = thrust
                    break
            else:
                if thrust < end_threshold:
                    self.rise_end_time = sample_time
                    self.rise_end_thrust = thrust
                    break

        # Calculate the time constant
        # See https://en.wikipedia.org/wiki/Rise_time
        self.time_constant = (self.rise_end_time - self.rise_start_time) / 2.197
        self.fitted_transition_start_time = self.rise_start_time + self.time_constant*np.log(0.9)
        self.response_lag = self.fitted_transition_start_time - self.transition_start_time
        return self.time_constant

    def get_voltages(self):
        return (self._original_voltage_times, self.motor_voltages)

def parse_data_log(log, ramp_settings):
    log_began = False
    log_end = False
    ramp_objects = []
    last_data_block = None
    new_data_block = []
    start_throttle = 0
    end_throttle = 0
    
    log_iterator = iter(log)
    for line in log_iterator:
        line = line.rstrip()

        if log_began:
            if (line == 'RESPONSE END' or line == 'STOP') and last_data_block is not None:
                # End of a response
                #print((start_throttle, end_throttle))
                new_ramp = Ramp(start_throttle, end_throttle, last_data_block, new_data_block, ramp_settings)
                ramp_objects.append(new_ramp)
            elif line == 'RESPONSE END' or line == 'STOP':
                # Haven't processed a starting block yet so can't make a ramp object
                pass
            elif line == 'RESPONSE START':
                # Line must be immediately followed by the start and end points
                next_line = next(log_iterator).rstrip()

                throttles_text = next_line.split(',')
                throttles = []
                for data in throttles_text:
                    throttles.append(float(data))
                
                start_throttle = throttles[0]
                end_throttle = throttles[1]

                last_data_block = new_data_block
                new_data_block = []
            else:
                # Line must be a datapoint line
                new_data_point_text = line.split(',')
                new_data_point = []
                for data in new_data_point_text:
                    new_data_point.append(float(data))
                    
                new_data_point = tuple(new_data_point)
                new_data_block.append((new_data_point))

        if line == 'START':
            # Start of a log
            log_began = True
            
        if line == 'STOP':
            break

    return ramp_objects

def plot_all_filtered(ramps, n, m):
    plot_all_responses(ramps, n, m, filtered_getter=Ramp.get_filtered_thrust)

def plot_all_voltages(ramps, n, m):
    plot_all_responses(ramps, n, m, voltage_getter=Ramp.get_voltages)

def plot_all_voltages_filtered(ramps, n, m):
    plot_all_responses(ramps, n, m, filtered_getter=Ramp.get_filtered_thrust, voltage_getter=Ramp.get_voltages)

def plot_all_responses(ramps, n, m, thrust_getter=Ramp.get_thrusts_and_times, filtered_getter=None, voltage_getter=None):
    plt.figure()
    for i in range(0, len(ramps)):
        (thrust_times , thrusts) = thrust_getter(ramps[i])
        (throttle_times, throttles) = ramps[i].get_throttles_and_times()
        start_thrust = ramps[i].get_start_thrust()
        end_thrust = ramps[i].get_end_thrust()
        ax1 = plt.subplot(n, m, i+1)
        ax2 = ax1.twinx()

        if filtered_getter is not None:
            (filtered_thrust_times , filtered_thrusts) = filtered_getter(ramps[i])
            ax1.plot(thrust_times, thrusts, 'b-',
                     filtered_thrust_times, filtered_thrusts, 'm--',
                     ramps[i].fitted_response_curve_times, ramps[i].fitted_response_thrust, 'c:',
                     ramps[i].rise_start_time, ramps[i]._filtered_thrusts_interpolator(ramps[i].rise_start_time), 'kx',
                     ramps[i].rise_end_time, ramps[i]._filtered_thrusts_interpolator(ramps[i].rise_end_time), 'kx')
        else:
            ax1.plot(thrust_times, thrusts, 'b-')

        ax1.axhline(y=start_thrust, linewidth=1, color='g')
        ax1.axhline(y=end_thrust, linewidth=1, color='g')


        if voltage_getter is not None:
            ax3 = ax1.twinx()
            (filtered_voltage_times, filtered_voltages) = voltage_getter(ramps[i])
            ax3.plot(filtered_voltage_times, filtered_voltages, 'k')
            ax3.axhline(y=ramps[i].start_voltage, linewidth=1, color='c')
            ax3.axhline(y=ramps[i].end_voltage, linewidth=1, color='c')
        else:
            ax2.plot(throttle_times, throttles, color='r')

def plot_groups(ramps, group_plotter=plot_all_responses):
    n = 4
    m = 5

    for i in range(0, len(ramps)-n*m, n*m):
        group_plotter(ramps[i:i+(n*m)], n, m)
        last_plot = i+(n*m)
        print('Finished plot: {}'.format(i+n*m-1))
    
    group_plotter(ramps[last_plot:], n, m)

def plot_all_fft(ramps, n, m):
    plt.figure()
    for i in range(0, len(ramps)):
        (freqs, power) = ramps[i].get_fft()
        ax1 = plt.subplot(n, m, i+1)
        ax1.plot(freqs, 20 * np.log10(abs(power)), color='b')

def calculate_response_lags(ramps):
    response_lags = np.array([ramp.response_lag for ramp in ramps])

    #print('Response lags')
    #print(response_lags)

    response_lag = np.median(response_lags)
    print('Average response lag: {} med: {} std dev: {}'.format(np.average(response_lags), response_lag, np.std(response_lags)))

    plt.figure()
    plt.hist(response_lags, bins='auto')

    return float(response_lag)

def create_voltage_to_thrust(ramps, settings):
    thrusts = [ramp.start_thrust for ramp in ramps] + [ramp.end_thrust for ramp in ramps]
    voltages = [ramp.start_voltage for ramp in ramps] + [ramp.end_voltage for ramp in ramps]

    z = np.polyfit(voltages, thrusts, deg=settings['degree'])
    voltage_to_thrust = np.poly1d(z)
    fit_voltages = np.linspace(0, np.max(voltages)*1.2, 100)

    z = np.polyfit(thrusts, voltages, deg=settings['degree'])
    thrust_to_voltage = np.poly1d(z)
    fit_thrusts = np.linspace(0, np.max(thrusts)*1.2, 100)

    plt.figure()
    plt.subplot(211)
    plt.plot(voltages, thrusts, 'bo', fit_voltages, voltage_to_thrust(fit_voltages), 'r-')

    plt.subplot(212)
    plt.plot(thrusts, voltages, 'bo', fit_thrusts, thrust_to_voltage(fit_thrusts), 'r-')

    return voltage_to_thrust, thrust_to_voltage
    
def create_time_constant_to_last_thrust_and_voltage(ramps, poly, settings):

    rising_thrust_order = settings['rising_edge_thrust_order']
    rising_voltage_order = settings['rising_edge_voltage_order']

    falling_thrust_order = settings['falling_edge_thrust_order']
    falling_voltage_order = settings['falling_edge_voltage_order']

    up_start_thrusts = [ramp.start_thrust for ramp in ramps if ramp.start_thrust < ramp.end_thrust]
    up_end_voltages = [ramp.end_voltage for ramp in ramps if ramp.start_thrust < ramp.end_thrust]
    up_time_constants = [ramp.time_constant for ramp in ramps if ramp.start_thrust < ramp.end_thrust]

    down_start_thrusts = [ramp.start_thrust for ramp in ramps if ramp.start_thrust >= ramp.end_thrust]
    down_end_voltages = [ramp.end_voltage for ramp in ramps if ramp.start_thrust >= ramp.end_thrust]
    down_time_constants = [ramp.time_constant for ramp in ramps if ramp.start_thrust >= ramp.end_thrust]

    start_thrusts = up_start_thrusts+down_start_thrusts
    end_voltages = up_end_voltages+down_end_voltages
    time_constants = up_time_constants+down_time_constants

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(up_start_thrusts, up_end_voltages, up_time_constants, 'b+')
    ax.scatter(down_start_thrusts, down_end_voltages, down_time_constants, 'b+')

    #least_squares_and_plot(time_constants, start_thrusts, end_voltages, 3, 2, ax)
    poly_rising = least_squares_and_plot(up_time_constants, up_start_thrusts, up_end_voltages, rising_thrust_order, rising_voltage_order, ax, poly=poly, plot_rising_only=True)
    poly_falling = least_squares_and_plot(down_time_constants, down_start_thrusts, down_end_voltages, falling_thrust_order, falling_voltage_order, ax, poly=poly, plot_rising_only=False)

    ax.set_xlabel('Thrusts (kg)')
    ax.set_ylabel('End Voltages (V)')
    ax.set_zlabel('Time Constant (tau)')

    return lambda thrust, voltage: find_time_constant_with_multiple_fits(thrust, voltage, poly_rising, poly_falling, poly)

def least_squares_and_plot(time_constants, start_thrusts, end_voltages, degree_thrust, degree_voltage, ax, poly=None, plot_rising_only=None):
    # Generate the A matrix
    A = generate_2d_polynomial_terms(start_thrusts, degree_thrust, end_voltages, degree_voltage)
    lsq_sol = lsq_linear(A, time_constants)
    c = lsq_sol.x.reshape(degree_thrust+1, degree_voltage+1)

    thrust_fit_points_count = 25
    voltage_fit_points_count = 25
    thrust_fit_points = np.linspace(min(start_thrusts)*0.9, max(start_thrusts), thrust_fit_points_count*1.1)
    voltage_fit_points = np.linspace(min(end_voltages)*0.9, max(end_voltages), voltage_fit_points_count*1.1)
    thrust_fit_mesh, voltage_fit_mesh = np.meshgrid(thrust_fit_points, voltage_fit_points)

    thrust_fit_mesh = thrust_fit_mesh.flatten()
    voltage_fit_mesh = voltage_fit_mesh.flatten()

    if plot_rising_only is not None:
        if plot_rising_only:
            thrust_fit_mesh, voltage_fit_mesh = zip(*[(x,y) for (x,y) in zip(thrust_fit_mesh, voltage_fit_mesh) if x <= poly(y)])
        else:
            thrust_fit_mesh, voltage_fit_mesh = zip(*[(x,y) for (x,y) in zip(thrust_fit_mesh, voltage_fit_mesh) if x > poly(y)])

    fitted = np.polynomial.polynomial.polyval2d(thrust_fit_mesh, voltage_fit_mesh, c)
    ax.plot_trisurf(thrust_fit_mesh, voltage_fit_mesh, fitted, alpha=0.5)
    return c

def find_time_constant_with_multiple_fits(thrusts, voltages, accelerating_time_constant_coefficients, braking_time_constant_coefficients, voltage_to_thrust):
    time_constants = []
    for (thrust, voltage) in zip(thrusts, voltages):
        if thrust <= voltage_to_thrust(voltage):
            time_constants.append(np.polynomial.polynomial.polyval2d(thrust, voltage, accelerating_time_constant_coefficients))
        else:
            time_constants.append(np.polynomial.polynomial.polyval2d(thrust, voltage, braking_time_constant_coefficients))
    return np.array(time_constants)

def generate_model_map(voltage_to_thrust, time_constant_function, settings):
    thrust_min = settings['thrust_min']
    thrust_max = settings['thrust_max']
    voltage_min = settings['voltage_min']
    voltage_max = settings['voltage_max']

    thrust_points_count = settings['thrust_points']
    voltage_points_count = settings['voltage_points']

    Ts = settings['timestep']

    thrust_points = np.linspace(thrust_min, thrust_max, thrust_points_count, endpoint=True)

    voltage_points = np.linspace(voltage_min, voltage_max, voltage_points_count, endpoint=True)

    voltage_points_mesh, thrust_points_mesh  = np.meshgrid(voltage_points, thrust_points)
    thrust_points_mesh = thrust_points_mesh.flatten()
    voltage_points_mesh = voltage_points_mesh.flatten()

    #t_p = [0.7111111111111111]
    #v_p = [9.333333333333332]

    #print time_constant_function(t_p, v_p)

    T = (1-np.exp(-Ts/time_constant_function(thrust_points_mesh, voltage_points_mesh))) * (voltage_to_thrust(voltage_points_mesh) - thrust_points_mesh) + thrust_points_mesh

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


if __name__ == "__main__":
    filename = sys.argv[1]

    with open(filename+'.yaml') as f:
        settings = yaml.load(f)

    with open(filename) as f:
        content = f.readlines()
    ramps = parse_data_log(content, settings['time_constant_calculation'])
    print('Finished parsing data')

    #plot_groups(ramps)
    #plot_groups(ramps, group_plotter=plot_all_filtered)
    #plot_groups(ramps, group_plotter=plot_all_voltages)

    #plot_groups(ramps, group_plotter=plot_all_fft)
    #plot_groups(ramps, group_plotter=plot_all_voltages_filtered)

    response_lag = calculate_response_lags(ramps)
    voltage_to_thrust_poly, thrust_to_voltage_poly = create_voltage_to_thrust(ramps, settings['voltage_to_thrust_estimator'])
    thrust_voltage_to_time_constant = create_time_constant_to_last_thrust_and_voltage(ramps, voltage_to_thrust_poly, settings['time_constant_fit'])

    voltage_to_jerk_model = generate_model_map(voltage_to_thrust_poly, thrust_voltage_to_time_constant, settings['voltage_to_jerk_estimator'])

    output_model = {'response_lag': response_lag,
                    'thrust_to_voltage': thrust_to_voltage_poly.coef.tolist(),
                    'voltage_to_jerk': voltage_to_jerk_model}

    with open(filename+'.output.yaml', 'w') as f:
        f.write(yaml.safe_dump(output_model))

    plt.show()
