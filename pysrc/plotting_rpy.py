#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped

import math
import time
import matplotlib.pyplot as plt

class PlottingRPY:
    def __init__(self):
        ## subscriber
        self.sub_vector = rospy.Subscriber("/rpy", Vector3Stamped, self.callbackVector)
        ## msg
        self.rpy = Vector3Stamped()
        ## list
        self.list_t = []
        self.list_r = []
        self.list_p = []
        ## time
        self.start_time = time.time()
        ## flag
        self.appended_new_msg = True
        ## parameter
        self.interval = 0.1
        self.ylim = 45.0
        self.shown_size = 100

        ## initialization
        self.line_r, self.line_p = self.initializePlot()
        ## loop
        self.mainLoop()

    def callbackVector(self, msg):
        self.rpy = msg
        self.appended_new_msg = False

    def initializePlot(self):
        plt.ion()   #interactive mode on
        plt.figure()
        ## empty
        self.list_t = [0 for i in range(self.shown_size)]
        self.list_r = [0 for i in range(self.shown_size)]
        self.list_p = [0 for i in range(self.shown_size)]
        ## roll
        plt.subplot(2, 1, 1)
        # plt.title("roll")
        plt.xlabel("time[s]")
        plt.ylabel("roll[deg]")
        plt.ylim(-self.ylim, self.ylim)
        plt.grid(True)
        line_r, = plt.plot(self.list_t, self.list_r)   #get line
        ## pitch
        plt.subplot(2, 1, 2)
        # plt.title("pitch")
        plt.xlabel("time[s]")
        plt.ylabel("pitch[deg]")
        plt.ylim(-self.ylim, self.ylim)
        plt.grid(True)
        line_p, = plt.plot(self.list_t, self.list_p)   #get line

        return line_r, line_p

    def mainLoop(self):
        while not rospy.is_shutdown():
            if not self.appended_new_msg:
                self.updatePlot()
                self.appended_new_msg = True

    def updatePlot(self):
        ## append
        t = time.time() - self.start_time
        self.list_t.append(t)
        self.list_r.append(self.rpy.vector.x/math.pi*180.0)
        self.list_p.append(self.rpy.vector.y/math.pi*180.0)
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
        plt.pause(self.interval)

def main():
    rospy.init_node('plotting_rpy', anonymous=True)

    plotting_rpy = PlottingRPY()

    rospy.spin()

if __name__ == '__main__':
    main()