#!/usr/bin/env python
import rospy
import numpy as np

class PidSettings(object):
    def __init__(self, settings_dict):
        self.kp = settings_dict['kp']
        self.ki = settings_dict['ki']
        self.kd = settings_dict['kd']
        self.accum_max = settings_dict['accumulator_max']
        self.accum_min = settings_dict['accumulator_min']
        self.accum_en_threshold = settings_dict['accumulator_enable_threshold']
        self.i_accumulator_initial_value = None

class PidController(object):
    def __init__(self, settings):
        self._p_gain = settings.kp
        self._i_gain = settings.ki
        self._d_gain = settings.kd
        self._i_accumulator_max = settings.accum_max
        self._i_accumulator_min = settings.accum_min
        self._i_accumulator_enable_threshold = settings.accum_en_threshold

        self._initialized = False
        self._last_time = None

        if settings.i_accumulator_initial_value is not None:
            self._i_accumulator = settings.i_accumulator_initial_value
        else:
            self._i_accumulator = 0.0

        self._last_current_value = 0.0
        self._setpoint = 0.0

    def reset_accumulator(self):
        self._i_accumulator = 0.0

    def set_setpoint(self, setpoint):
        self._setpoint = setpoint

    def get_accumulator(self):
        return self._i_accumulator

    def update(self, current_value, time, log_debug):
        '''
        Updates PID controller

        Args:
            current_value: current value of whatever you are controlling on
            time: current time; use rospy.get_time()
            log_debug: enables verbose debugging

        Returns:
            success: whether or not a response could be calculated
            response is the filter output
        '''
        response = 0

        if self._last_time is None:
            self._last_time = time
            self._initialized = True
            return True, response

        if time == self._last_time:
            rospy.logwarn(
                    'Time passed in to PidController is equal to last time.')
            return False, response

        if time < self._last_time:
            rospy.logwarn(
                    'Time passed in to PidController is less than the last time.')
            return False, response

        if not np.isfinite(current_value):
            rospy.logwarn(
                    'Invalid argument to PidControllor.update (current_value = {}).'.format(current_value))
            return False, response

        difference = self._setpoint - current_value
        p_term = self._p_gain * difference
        response = p_term

        time_delta = (time - self._last_time).to_sec()

        if np.absolute(difference) < self._i_accumulator_enable_threshold:
            self._i_accumulator = (self._i_accumulator
                                + self._i_gain * difference * time_delta)

            if log_debug:
                log = ('Set accumulator to ' + str(self._i_accumulator) +
                        ', i_gain to ' + str(self._i_gain) +
                        ', difference is ' + str(difference) +
                        ', time delta is ' + str(time_delta))
                rospy.logwarn(log)

            self._i_accumulator = max(min(self._i_accumulator_max,
                                          self._i_accumulator),
                                          self._i_accumulator_min)

            if log_debug:
                log = ('Accumulator clamped to ' + str(self._i_accumulator) +
                        ' (min: ' + str(self._i_accumulator_min) +
                        ', max: ' + str(self._i_accumulator_max) + ')')
                rospy.logwarn(log)
        else:
            if log_debug:
                log = ('Ignoring difference ' + str(difference) +
                        ' above threshold ' + str(self._i_accumulator_enable_threshold))
                rospy.logwarn(log)

        response = response + self._i_accumulator

        derivative = (current_value - self._last_current_value) / time_delta

        d_term = self._d_gain * derivative
        response = response - d_term

        if log_debug:
            log = ('p: ' + str(p_term) +
                    ' I: ' + str(self._i_accumulator) +
                    ' D: ' + str(d_term))

            rospy.logwarn(log)

        self._last_current_value = current_value
        self._last_time = time

        if not np.isfinite(response):
            rospy.logwarn('Invalid result from PidController.update (response = {})'.format(response))
            return False, response
        else:
            return True, response
