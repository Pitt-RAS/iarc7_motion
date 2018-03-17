import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
import scipy.signal as signal
import math
import os

FIGURE_SIZE = (18.0, 12.0)

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

class RampResponse(object):
    def __init__(self, command, ramp_rate, data):
        self.ramp_rate = ramp_rate * 50
        self.command = command
        self.data = data

        # Try to find a file format, if no string assume 1.0
        # Assume format is 1.1 (includes expected thrust at end)
        self.THRUST_COLUMN = 0
        self.THRUST_TIME_COLUMN = 1
        self.CURRENT_COLUMN = 2
        self.CURRENT_STAMP_COLUMN = 3
        self.VOLTAGE_COLUMN = 4
        self.VOLTAGE_STAMP_COLUMN = 5
        self.WATTAGE_COLUMN = 6
        self.THROTTLE_COLUMN = 7
        self.THROTTLE_TIME_COLUMN = 8
        self.EXPECTED_THRUST_COLUMN = 10
        self.EXPECTED_THRUST_TIME_COLUMN = 11

        # Use voltage/current measurements to find the start time since it is the fastest updated
        self._start_time = self.data[0][self.THROTTLE_TIME_COLUMN]/1000000.0

        self._expected_thrust_offset = 0.0

        (self._original_thrust_times , self._original_thrusts) = remove_consecutive_duplicates([x[self.THRUST_TIME_COLUMN]/1000000.0 for x in self.data], [x[self.THRUST_COLUMN] for x in self.data])
        self._original_thrust_times = [x - self._start_time for x in self._original_thrust_times]

        (self.current_times, self.current) = zip(*[(x[self.CURRENT_STAMP_COLUMN]/1000000.0, x[self.CURRENT_COLUMN]) for x in self.data])
        self.current_times = [x - self._start_time for x in self.current_times]

        (self.wattage_times, self.wattage) = zip(*[(x[self.CURRENT_STAMP_COLUMN]/1000000.0, x[self.WATTAGE_COLUMN]) for x in self.data])
        self.wattage_times = [x - self._start_time for x in self.wattage_times]

        (self._original_throttle_times, self._original_throttles) = remove_consecutive_duplicates([x[self.THROTTLE_TIME_COLUMN]/1000000.0 for x in self.data], [x[self.THROTTLE_COLUMN] for x in self.data])
        self._original_throttle_times = [x - self._start_time for x in self._original_throttle_times]

        (self._original_voltage_times , self._original_battery_voltages) = [x[self.VOLTAGE_STAMP_COLUMN]/1000000.0 for x in self.data], [x[self.VOLTAGE_COLUMN] for x in self.data]
        self._original_voltage_times = [x - self._start_time for x in self._original_voltage_times]
        self.throttle_interp = interp1d(self._original_throttle_times, self._original_throttles, kind='zero', bounds_error=False, fill_value=(self._original_throttles[0], self._original_throttles[-1]))
        self.motor_voltages = np.array([battery_voltage*self.throttle_interp(time)/100.0 for (time, battery_voltage) in zip(self._original_voltage_times, self._original_battery_voltages)])

        (self._original_expected_thrust_times , self._original_expected_thrust) = [x[self.EXPECTED_THRUST_TIME_COLUMN]/1000000.0 for x in self.data], [x[self.EXPECTED_THRUST_COLUMN] for x in self.data]
        self._original_expected_thrust_times = [x - self._start_time + self._expected_thrust_offset for x in self._original_expected_thrust_times]

        self.find_time_offset()

        self._shifted_expected_thrust_times = [x - self.time_offset for x in self._original_expected_thrust_times]

        self.find_rms_error()

        self.peak_gain = ((np.max(self._original_thrusts) - np.min(self._original_thrusts))
                          / (np.max(self._original_expected_thrust) - np.min(self._original_expected_thrust)))
        self.max_error = np.max(np.abs(self.thrust_error))

    def find_time_offset(self):        
        self.thrust_interp = interp1d(self._original_thrust_times,
                                      self._original_thrusts,
                                      kind='linear',
                                      bounds_error=False,
                                      fill_value=(self._original_thrusts[0], self._original_thrusts[-1]))
        self.expected_thrust_interp = interp1d(self._original_expected_thrust_times,
                                               self._original_expected_thrust,
                                               kind='linear',
                                               bounds_error=False,
                                               fill_value=(self._original_expected_thrust[0], self._original_expected_thrust[-1]))

        self._correllation_resolution = 0.001
        num_points = int((self._original_thrust_times[-1] - self._original_thrust_times[0])/self._correllation_resolution)
        interp_times = np.linspace(self._original_thrust_times[0],self._original_thrust_times[-1], num=num_points, endpoint=False)
        self.interp_times = interp_times

        interpolated_thrusts = self.thrust_interp(interp_times)
        interpolated_expected_thrusts = self.expected_thrust_interp(interp_times)

        self.interpolated_thrusts = interpolated_thrusts
        self.interpolated_expected_thrusts = interpolated_expected_thrusts

        self.correlation = signal.correlate(interpolated_expected_thrusts,
                                            interpolated_thrusts,
                                            mode='full')

        timestep = (interp_times[1] - interp_times[0])

        max_forward_correlation_time = math.ceil(0.0/timestep)
        total_correlations = len(interpolated_thrusts) + max_forward_correlation_time
        self.correlation = self.correlation[:total_correlations+1]

        index = np.argmax(self.correlation)

        self.time_offset = (index - len(interpolated_thrusts) + 1) * timestep

        correlation_time_min = -(len(self.correlation)-1)*timestep
        correlation_time_max = max_forward_correlation_time
        self.correlation_times = np.linspace(correlation_time_min, correlation_time_max, num=len(self.correlation))

    def find_rms_error(self):
        # Used trimmed dataset for RMS cut off bottom and end
        percent_trim = 0.05
        start_index = int(math.ceil(percent_trim*len(self._shifted_expected_thrust_times)))
        end_index = int(math.ceil((1-percent_trim)*len(self._shifted_expected_thrust_times)))

        interpolated_thrust = self.thrust_interp(self._shifted_expected_thrust_times)
        self.thrust_error = self._original_expected_thrust[start_index:end_index] - interpolated_thrust[start_index:end_index]
        self.thrust_error_times = self._shifted_expected_thrust_times[start_index:end_index]
        self.shifted_thrust_rms_error = (np.sum(np.square(self.thrust_error))/len(self.thrust_error))**0.5

    def __str__(self):
        return 'Ramp Response Object for {} kg/s'.format(self.ramp_rate)

    def get_thrusts_and_times(self):
        return (self._original_thrust_times, self._original_thrusts)

    def get_throttles_and_times(self):
        return (self._original_throttle_times, self._original_throttles)    

    def get_voltages(self):
        return (self._original_voltage_times, self.motor_voltages)

    def get_expected_thrusts_and_times(self):
        return (self._original_expected_thrust_times, self._original_expected_thrust)

    def get_shifted_expected_thrusts_and_times(self):
        return (self._shifted_expected_thrust_times, self._original_expected_thrust)

    def get_interpolated_thrusts_and_times(self):
        return (self.interp_times, self.interpolated_thrusts)

    def get_interpolated_expected_thrusts_and_times(self):
        return (self.interp_times, self.interpolated_expected_thrusts)

    def get_interpolated_thrust_error(self):
        return (self.thrust_error_times, self.thrust_error)

    def get_current(self):
        return (self.current_times, self.current)

    def get_wattage(self):
        return (self.wattage_times, self.wattage)

    def get_throttle(self):
        return (self._original_throttle_times, self._original_throttles)

def parse_data_log(log):
    log_began = False
    ramp_groups = []
    ramp_objects = []
    new_data_block = []
    ramp_rate = 0
    command_number = 0
    
    log_iterator = iter(log)
    for line in log_iterator:
        line = line.rstrip()

        if log_began:
            if (line == 'RESPONSE END' or line == 'STOP'):
                # End of a response
                new_ramp = RampResponse(command_number, ramp_rate, new_data_block)
                ramp_objects.append(new_ramp)
            elif line == 'RESPONSE START':
                # Line must be immediately followed by the start and end points
                next_line = next(log_iterator).rstrip()

                ramp_rate = float(next_line)

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
            try:
                next_line = next(log_iterator).rstrip()
                command_number = float(next_line)
                next_line = next(log_iterator).rstrip()
                ramp_rate = float(next_line)
                command_number = int(command_number)
            except Exception as e:
                ramp_rate = command_number
                command_number = 0

        if line == 'STOP':
            ramp_groups.append(ramp_objects)

            log_began = False
            ramp_objects = []
            new_data_block = []
            ramp_rate = 0
            command_number = 0

    return ramp_groups

def plot_all_responses(ramps, n, m, thrust_getter=RampResponse.get_thrusts_and_times,
                                    voltage_getter=RampResponse.get_voltages,
                                    expected_thrust_getter=RampResponse.get_expected_thrusts_and_times,
                                    thrust_error_getter=RampResponse.get_interpolated_thrust_error):
    plt.figure(figsize=FIGURE_SIZE)
    for i in range(0, len(ramps)):

        (thrust_times , thrusts) = thrust_getter(ramps[i])
        (expected_thrust_times, expected_thrusts) = expected_thrust_getter(ramps[i])
        ax1 = plt.subplot(n, m, i+1)
        ax1.plot(thrust_times, thrusts, 'b',
                thrust_times, thrusts, 'bo',
                 expected_thrust_times, expected_thrusts, 'ro')

        #ax3 = ax1.twinx()
        #(filtered_voltage_times, filtered_voltages) = voltage_getter(ramps[i])
        #ax3.plot(filtered_voltage_times, filtered_voltages, 'k')

        ax4 = ax1.twinx()
        (thrust_error_times, thrust_error) = thrust_error_getter(ramps[i])
        ax4.plot(thrust_error_times, thrust_error, 'c')

        set_title(ramps[i])

def plot_all_responses_shifted(ramps, n, m):
    plot_all_responses(ramps, n, m, expected_thrust_getter=RampResponse.get_shifted_expected_thrusts_and_times)
    #plt.suptitle('All measured thrusts shifted left {} ms'.format(expected_offset))

def plot_all_responses_interpolated(ramps, n, m):
    plot_all_responses(ramps, n, m, thrust_getter=RampResponse.get_interpolated_thrusts_and_times,
                                    expected_thrust_getter=RampResponse.get_interpolated_expected_thrusts_and_times)

def plot_all_correlations(ramps, n, m):

    plt.figure(figsize=FIGURE_SIZE)
    for i in range(0, len(ramps)):

        ax1 = plt.subplot(n, m, i+1)
        ax1.plot(ramps[i].correlation_times, ramps[i].correlation, 'b')
        set_title(ramps[i])

def set_title(ramp):
    plt.title('Ramp Response rate: {0:.2f} kg/s \nlag: {1:.2f} ms RMS error: {2:.2f} kg\nMax gain: {3:.2f} Max error: {4:.2f}'.format(ramp.ramp_rate,
                                                                                              ramp.time_offset*1000,
                                                                                              ramp.shifted_thrust_rms_error,
                                                                                              ramp.peak_gain,
                                                                                              ramp.max_error))

def plot_power_consumption(ramps, n, m):
    plt.figure(figsize=FIGURE_SIZE)
    for i in range(0, len(ramps)):

        (current_times , current) = ramps[i].get_current()
        ax1 = plt.subplot(n, m, i+1)
        #ax1.plot(current_times, current, 'g')

        (wattage_times , wattage) = ramps[i].get_wattage()
        #ax2 = ax1.twinx()
        #ax2.plot(wattage_times, wattage, 'r')

        (throttle_times , throttle) = ramps[i].get_throttle()
        ax3 = ax1.twinx()
        ax3.plot(throttle_times, throttle, 'k')

def plot_groups(ramps, group_plotter=plot_all_responses):

    if len(ramps) == 1:
        n = 1
        m = 1
    else:
        n = 2
        m = 3

    last_plot = 0
    for i in range(0, len(ramps)-n*m, n*m):
        group_plotter(ramps[i:i+(n*m)], n, m)
        last_plot = i+(n*m)
        print('Finished plot: {}'.format(i+n*m-1))

    group_plotter(ramps[last_plot:], n, m)

    titles = ['Dynamic Model Ramp Response 25% margins',
              'Static Model Ramp Response 25% margins',
              'Dynamic Model Ramp Response 5% margins',
              'Static Model Ramp Response 5% margins',
              'Dynamic Model Steady State Accuracy',
              'Static Model Steady State Accuracy']
    plt.suptitle(titles[ramps[0].command-1])

def find_expected_offset(ramps):
    offsets = [ramp.time_offset for ramp in ramps]
    return np.median(offsets)

if __name__ == "__main__":
    filename = sys.argv[1]

    with open(filename) as f:
        content = f.readlines()
    ramp_groups = parse_data_log(content)
    print('Finished parsing data')

    for ramp_responses in ramp_groups:

        plot_groups(ramp_responses)

        #plot_groups(ramp_responses, group_plotter=plot_all_correlations)

        expected_offset = find_expected_offset(ramp_responses)

        #plot_groups(ramp_responses, group_plotter=plot_all_responses_shifted)

        #plot_groups(ramp_responses, group_plotter=plot_power_consumption)

        #plot_groups(ramp_responses, group_plotter=plot_all_responses_interpolated)

    directory = os.path.split(filename)[0]
    model_name = os.path.split(filename)[1].split('.txt')[0]

    save_dir = directory+'/'+model_name
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    figs = [plt.figure(n) for n in plt.get_fignums()]
    i = 1
    for fig in figs:
        fig.savefig(save_dir+'/'+ str(i) +'.png', format='png')
        i = i + 1

    plt.show()
