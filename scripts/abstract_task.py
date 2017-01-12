#!/usr/bin/env python
import rospy

class AbstractTask:

    # Abstract method
    def get_preferred_velocity(self):
        raise NotImplementedError("Subclass must implement abstract method")
