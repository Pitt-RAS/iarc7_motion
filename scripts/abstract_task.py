#!/usr/bin/env python
import rospy

class AbstractTask:

    # Abstract method
    def get_preferred_velocity(self):
        raise NotImplementedError("Subclass must implement abstract method")

    # Abstract method
    def cancel(self):
        raise NotImplementedError("Subclass must implement abstract method")

    # Abstract method
    def is_done(self):
        raise NotImplementedError("Subclass must implement abstract method")

    # Abstract method
    def is_aborted(self):
        raise NotImplementedError("Subclass must implement abstract method")

    # Abstract method
    def get_result(self):
        raise NotImplementedError("Subclass must implement abstract method")
