# Implemented with "extensive" help from https://github.com/ros2/message_filters/blob/rolling/src/message_filters/__init__.py
# BSD 3-Clause License yadda yadda
from message_filters import SimpleFilter
import threading
from functools import reduce

from builtin_interfaces.msg import Time as TimeMsg
import rclpy
from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.time import Time

class ResistantTimeSynchronizer(SimpleFilter):
    def __init__(self, fs, queue_size, toggle_tolerance, prune_aggressively = False):
        SimpleFilter.__init__(self)
        self.connectInput(fs)
        self.queue_size = queue_size
        self.lock = threading.Lock()
        self.toggle_tolerance = toggle_tolerance
        self.toggles = None
        self.prev_timeStamp = None
        self.firstStamp = None
        self.prune_aggro = prune_aggressively
            # prune_aggressively allows toggles to be turned off without an actual response from the camera in question
            # it should be on if we need to tolerate the case where a camera is fully broken

    def connectInput(self, fs):
        self.queues = [{} for f in fs]
        self.toggles = [True for _ in fs]
        self.prev_timeStamp = [None for _ in fs]
        self.firstStamp = None
        self.input_connections = [
            f.registerCallback(self.add, q, i_q)
            for i_q, (f, q) in enumerate(zip(fs, self.queues))]

    def add(self, msg, my_queue, my_queue_index=None):
        self.lock.acquire()
        stamp = Time.from_msg(msg.header.stamp)
        my_queue[stamp.nanoseconds] = msg
        if self.firstStamp == None:
            self.firstStamp = stamp.nanoseconds
        if not self.prune_aggro:
            if self.prev_timeStamp[my_queue_index] != None and stamp.nanoseconds - self.prev_timeStamp[my_queue_index] > self.toggle_tolerance * 1000000000:
                self.toggles[my_queue_index] = False
            else:
                self.toggles[my_queue_index] = True
            self.prev_timeStamp[my_queue_index] = stamp.nanoseconds
        else:
            self.prev_timeStamp[my_queue_index] = stamp.nanoseconds
            for i in range(0,len(self.toggles)):
                self.toggles[i] = (stamp.nanoseconds - (self.firstStamp if self.prev_timeStamp[i] == None else self.prev_timeStamp[i]) < self.toggle_tolerance * 1000000000)

        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        # common is the set of timestamps that occur in all queues
        activeIndices = [i for i,on in enumerate(self.toggles) if on]
        common = reduce(set.intersection, [set(self.queues[i]) for i in activeIndices])
        for t in sorted(common):
            # msgs is list of msgs (one from each queue) with stamp t
            msgs = [self.queues[i][t] for i in activeIndices]
            self.signalMessage(*msgs)
            for i in activeIndices:
                del self.queues[i][t]
        self.lock.release()