#!/usr/bin/env python
import rospy

class AbstractTask:

    # Abstract method
    def get_desired_command(self):
        raise NotImplementedError("Subclass must implement abstract method")

    # Abstract method
    def cancel(self):
        raise NotImplementedError("Subclass must implement abstract method")

    # Abstract method
    def send_transition(self):
    	raise NotImplementedError("Subclass must implement abstract method")
