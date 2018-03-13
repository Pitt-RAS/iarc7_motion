#!/usr/bin/env python

'''
AbstractTask: abstract class defining tasks, which 
are defined motion actions or commands.

'''

import rospy

from task_utilities.task_topic_buffer import TaskTopicBuffer

class AbstractTask(object):

    topic_buffer = None

    def __init__(self):
        if AbstractTask.topic_buffer is None:
            AbstractTask.topic_buffer = TaskTopicBuffer()
        self.topic_buffer = AbstractTask.topic_buffer

    # Abstract method
    def get_desired_command(self):
        '''
        This is the "main" function of a task. This method is called 
        to get the next command desired by the task that the rest of 
        the stack is to implement/act upon. 

        Returns: 
            TaskState: instance of whatever state the task is in
            Task Command: desired command of the task

        '''
        raise NotImplementedError("Subclass must implement abstract method")

    # Abstract method
    def cancel(self):
        '''
        Cancels the task.

        '''
        raise NotImplementedError("Subclass must implement abstract method")

    # Abstract method
    def set_incoming_transition(self):
        '''
        Public method for an action coordinator to send transition information

        Args: 
            transition: instance of some form of transition information

        '''
        raise NotImplementedError("Subclass must implement abstract method")
