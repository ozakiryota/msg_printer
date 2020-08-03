#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped

import math
import time
import matplotlib.pyplot as plt

class CompareRPY:
    def __init__(self):
        ## subscriber
        self.sub_truth = rospy.Subscriber("/truth/rpy", Vector3Stamped, self.callbackTruth)
        self.sub_estimation = rospy.Subscriber("/estimation/rpy", Vector3Stamped, self.callbackEstimation)
        ## publisher
        self.pub_error = rospy.Publisher("/error/rpy", Vector3Stamped, queue_size=1)
        ## msg
        self.truth = Vector3Stamped()
        self.estimation = Vector3Stamped()
        self.error = Vector3Stamped()
        ## list
        self.list_t = []
        self.list_r = []
        self.list_p = []
        ## line
        # self.line_r = None    #primitive
        # self.line_p = None    #primitive
        ## flag
        self.done_initialization = False
        self.got_first_estimation = False
        ## time
        self.start_time = time.time()
        ## parameter
        self.ylim = 30.0
        self.shown_size = 50

        ## initialization
        self.line_r, self.line_p = self.initializePlot()

    def initializePlot(self):
        plt.ion()   #interactive mode on
        plt.figure()
        ## empty
        self.list_t = [0 for i in range(self.shown_size)]
        self.list_r = [0 for i in range(self.shown_size)]
        self.list_p = [0 for i in range(self.shown_size)]
        ## roll
        plt.subplot(2, 1, 1)
        plt.title("roll")
        plt.xlabel("time[s]")
        plt.ylabel("roll[deg]")
        plt.ylim(-self.ylim, self.ylim)
        plt.grid(True)
        line_r, = plt.plot(self.list_t, self.list_r)   #get line
        ## pitch
        plt.subplot(2, 1, 2)
        plt.title("pitch")
        plt.xlabel("time[s]")
        plt.ylabel("pitch[deg]")
        plt.ylim(-self.ylim, self.ylim)
        plt.grid(True)
        line_p, = plt.plot(self.list_t, self.list_p)   #get line
        ## flag
        self.done_initialization = True

        return line_r, line_p

    def callbackTruth(self, msg):
        self.truth = msg
        if not self.got_first_estimation:
            self.got_first_estimation = True

    def callbackEstimation(self, msg):
        self.estimation = msg
        if self.done_initialization and self.got_first_estimation:
            self.computeError()
            self.publication()
            self.updatePlot()
    
    def computeError(self):
        self.error.vector.x = self.computeAngleDiff(
            self.truth.vector.x,
            self.estimation.vector.x
        )
        self.error.vector.y = self.computeAngleDiff(
            self.truth.vector.y,
            self.estimation.vector.y
        )
        self.error.vector.z = self.computeAngleDiff(
            self.truth.vector.z,
            self.estimation.vector.z
        )
    
    def computeAngleDiff(self, x, y):
        diff = math.atan2(math.sin(x - y), math.cos(x - y))
        return diff

    def updatePlot(self):
        ## append
        t = time.time() - self.start_time
        self.list_t.append(t)
        self.list_r.append(self.truth.vector.x)
        self.list_p.append(self.truth.vector.y)
        ## pop
        self.list_t.pop(0)
        self.list_r.pop(0)
        self.list_p.pop(0)
        ## roll
        plt.subplot(2,1,1)
        self.line_r.set_xdata(self.list_t)
        self.line_r.set_ydata(self.list_r)
        plt.xlim(min(self.list_t), max(self.list_t))
        ## pitch
        plt.subplot(2,1,2)
        self.line_p.set_xdata(self.list_t)
        self.line_p.set_ydata(self.list_p)
        plt.xlim(min(self.list_t), max(self.list_t))
        ## draw
        plt.draw()
        plt.pause(0.0001)

    def publication(self):
        self.error.header = self.estimation.header
        self.pub_error.publish(self.error)

def main():
    rospy.init_node('compare_rpy', anonymous=True)

    compare_rpy = CompareRPY()

    rospy.spin()

if __name__ == '__main__':
    main()
