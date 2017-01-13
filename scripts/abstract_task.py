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
    def done(self):
        raise NotImplementedError("Subclass must implement abstract method")

    # Abstract method
    def aborted(self):
        raise NotImplementedError("Subclass must implement abstract method")
