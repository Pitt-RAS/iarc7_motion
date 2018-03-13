#!/usr/bin/env python

import numpy as np
import rospy

class PidController(object):
    def __init__(self,
                 p_gain,
                 i_gain,
                 d_gain,
                 i_accumulator_max,
                 i_accumulator_min,
                 i_accumulator_enable_threshold):
        self._p_gain = p_gain
        self._i_gain = i_gain
        self._d_gain = d_gain
        self._initialized = False
        self._i_accumulator = 0.0
        self._last_current_value = 0.0
        self._last_time = None
        self._setpoint = 0.0
        self._i_accumulator_max = i_accumulator_max
        self._i_accumulator_min = i_accumulator_min
        self._i_accumulator_enable_threshold = i_accumulator_enable_threshold

    def reset_accumulator(self):
        self._i_accumulator = 0.0

    def set_setpoint(self, setpoint):
        self._setpoint = setpoint

    def update(self, current_value, time, log_debug):
        '''
        to get current time 'time' pass in rospy.get_time() in float seconds
        Current_value is what to pass in to filter
        Response is the filter output
        log_debug enables verbose debugging with True boolean
        '''
        response = 0

        if self._last_time is None:
            self._last_time = time
            rospy.logwarn("HERE")
            return True, response
        elif time == self._last_time:
            rospy.logwarn(
                    'Time passed in to PidController is equal to last time.')
            return False, response

        if time < self._last_time:
            rospy.logwarn(
                    'Time passed in to PidController is less than the last time.')
            return False, response

        if not np.isfinite(current_value):
            rospy.logwarn(
                    'Invalid argument to PidControllor.update (current_value = %f).',
                    current_value)
            return False, response

        difference = self._setpoint - current_value
        p_term = self._p_gain * difference
        response = p_term

        if not self._initialized:
            self._initialized = True

        else:
            time_delta = (time - self._last_time).to_sec()

            if np.absolute(difference) < self._i_accumulator_enable_threshold:
                self._i_accumulator = (self._i_accumulator
                                    + self._i_gain * difference * time_delta)

                if log_debug:
                    rospy.logwarn(
                            "Set accumulator to %f (i_gain %f) (difference %f) (time_delta %f)",
                            self._i_accumulator,
                            self._i_gain,
                            difference,
                            time_delta)

                self._i_accumulator = max(min(self._i_accumulator_max,
                                              self._i_accumulator),
                                          self._i_accumulator_min)

                if log_debug:
                    rospy.logwarn(
                            "Accumulator clamped to %f (min %f, max %f)",
                            self._i_accumulator,
                            self._i_accumulator_min,
                            self._i_accumulator_max)
            else:
                if log_debug:
                    rospy.logwarn(
                            "Ignoring difference %f above threshold %f",
                            difference,
                            self._i_accumulator_enable_threshold);

            response = response + self._i_accumulator

            derivative = (current_value - self._last_current_value) / time_delta

            d_term = self._d_gain * derivative
            response = response - d_term

            if log_debug:
                rospy.logwarn("p %f i %f d %f",
                              p_term,
                              self._i_accumulator,
                              d_term)

        self._last_current_value = current_value
        self._last_time = time

        if not np.isfinite(response):
            rospy.logwarn(
                    "Invalid result from PidController.update (response = %f)",
                    response)
            return False, response
        else:
            return True, response
