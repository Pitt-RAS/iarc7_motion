import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

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
    def __init__(self, ramp_rate, data):
        self.ramp_rate = ramp_rate
        self.data = data

        # Try to find a file format, if no string assume 1.0
        # Assume format is 1.1 (includes expected thrust at end)
        self.THRUST_COLUMN = 0
        self.THRUST_TIME_COLUMN = 1
        self.VOLTAGE_COLUMN = 4
        self.VOLTAGE_STAMP_COLUMN = 5
        self.THROTTLE_COLUMN = 7
        self.THROTTLE_TIME_COLUMN = 8
        self.EXPECTED_THRUST_COLUMN = 10
        self.EXPECTED_THRUST_TIME_COLUMN = 11

        # Use voltage/current measurements to find the start time since it is the fastest updated
        self._start_time = self.data[0][self.THROTTLE_TIME_COLUMN]/1000000.0

        (self._original_thrust_times , self._original_thrusts) = remove_consecutive_duplicates([x[self.THRUST_TIME_COLUMN]/1000000.0 for x in self.data], [x[self.THRUST_COLUMN] for x in self.data])
        self._original_thrust_times = [x - self._start_time for x in self._original_thrust_times]

        (self._original_throttle_times, self._original_throttles) = remove_consecutive_duplicates([x[self.THROTTLE_TIME_COLUMN]/1000000.0 for x in self.data], [x[self.THROTTLE_COLUMN] for x in self.data])
        self._original_throttle_times = [x - self._start_time for x in self._original_throttle_times]

        (self._original_voltage_times , self._original_battery_voltages) = [x[self.VOLTAGE_STAMP_COLUMN]/1000000.0 for x in self.data], [x[self.VOLTAGE_COLUMN] for x in self.data]
        self._original_voltage_times = [x - self._start_time for x in self._original_voltage_times]
        self.throttle_interp = interp1d(self._original_throttle_times, self._original_throttles, kind='zero', bounds_error=False, fill_value=(self._original_throttles[0], self._original_throttles[-1]))
        self.motor_voltages = np.array([battery_voltage*self.throttle_interp(time)/100.0 for (time, battery_voltage) in zip(self._original_voltage_times, self._original_battery_voltages)])

        (self._original_expected_thrust_times , self._original_expected_thrust) = [x[self.EXPECTED_THRUST_TIME_COLUMN]/1000000.0 for x in self.data], [x[self.EXPECTED_THRUST_COLUMN] for x in self.data]
        self._original_expected_thrust_times = [x - self._start_time for x in self._original_expected_thrust_times]

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

def parse_data_log(log):
    log_began = False
    ramp_objects = []
    new_data_block = []
    ramp_rate = 0
    
    log_iterator = iter(log)
    for line in log_iterator:
        line = line.rstrip()

        if log_began:
            if (line == 'RESPONSE END' or line == 'STOP'):
                # End of a response
                new_ramp = RampResponse(ramp_rate, new_data_block)
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
            
        if line == 'STOP':
            break

    return ramp_objects

def plot_all_responses(ramps, n, m, thrust_getter=RampResponse.get_thrusts_and_times,
                                    voltage_getter=RampResponse.get_voltages,
                                    expected_thrust_getter=RampResponse.get_expected_thrusts_and_times):
    plt.figure()
    for i in range(0, len(ramps)):

        (thrust_times , thrusts) = thrust_getter(ramps[i])
        ax1 = plt.subplot(n, m, i+1)
        ax1.plot(thrust_times, thrusts, 'b-')

        (throttle_times, throttles) = ramps[i].get_throttles_and_times()
        ax2 = ax1.twinx()
        ax2.plot(throttle_times, throttles, color='r')

        ax3 = ax1.twinx()
        (filtered_voltage_times, filtered_voltages) = voltage_getter(ramps[i])
        ax3.plot(filtered_voltage_times, filtered_voltages, 'k')

        plt.title('Ramp Response rate: {} kg/s'.format(ramps[i].ramp_rate))

def plot_groups(ramps, group_plotter=plot_all_responses):
    n = 4
    m = 5

    for i in range(0, len(ramps)-n*m, n*m):
        group_plotter(ramps[i:i+(n*m)], n, m)
        last_plot = i+(n*m)
        print('Finished plot: {}'.format(i+n*m-1))
    
    group_plotter(ramps[last_plot:], n, m)

if __name__ == "__main__":
    filename = sys.argv[1]

    with open(filename) as f:
        content = f.readlines()
    ramp_responses = parse_data_log(content)
    print('Finished parsing data')

    plot_groups(ramp_responses)

    plt.show()
