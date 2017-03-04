#!/usr/bin/env python

class TaskRunning(object):
    def __init__(self):
        pass

class TaskDone(object):
    def __init__(self):
        pass

class TaskCanceled(object):
    def __init__(self):
        pass

class TaskAborted(object):
    def __init__(self, msg=None):
        self.msg = msg

class TaskFailed(object):
    def __init__(self, msg=None):
        self.msg = msg

