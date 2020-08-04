#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped

import math
import time
import matplotlib.pyplot as plt
import numpy as np

class CompareRPY:
    def __init__(self):
        print("--- compare_rpy ---")
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
        self.list_truth_r = []
        self.list_truth_p = []
        self.list_estimation_r = []
        self.list_estimation_p = []
        self.list_error_rp = []
        ## line
        self.line_truth_r = None
        self.line_truth_p = None
        self.line_estimation_r = None
        self.line_estimation_p = None
        ## time
        self.start_time = time.time()
        ## flag
        self.got_first_truth = False
        self.got_new_msg = False
        ## parameter
        self.erase_old_data = rospy.get_param("/erase_old_data", True)
        print("self.erase_old_data = ", self.erase_old_data)
        self.interval = 0.1
        self.ylim = 45.0
        self.shown_size = 100

        ## initialization
        self.initializePlot()
        ## loop
        self.mainLoop()

    def callbackTruth(self, msg):
        self.truth = msg
        self.got_first_truth = True

    def callbackEstimation(self, msg):
        self.estimation = msg
        if self.got_first_truth:
            self.computeError()
            self.publication()
            self.got_new_msg = True
    
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
        ## append
        self.list_error_rp.append([self.error.vector.x, self.error.vector.y])
        ## print
        arr_error_rp = np.array(self.list_error_rp)
        print("arr_error_rp.shape = ", arr_error_rp.shape)
        print("arr_error_rp.mean(axis=0) = ", arr_error_rp.mean(axis=0))
    
    def computeAngleDiff(self, x, y):
        diff = math.atan2(math.sin(x - y), math.cos(x - y))
        return diff

    def publication(self):
        self.error.header = self.estimation.header
        self.pub_error.publish(self.error)

    def initializePlot(self):
        plt.ion()   #interactive mode on
        plt.figure()
        ## empty
        self.list_t = [0 for i in range(self.shown_size)]
        self.list_truth_r = [0 for i in range(self.shown_size)]
        self.list_truth_p = [0 for i in range(self.shown_size)]
        self.list_estimation_r = [0 for i in range(self.shown_size)]
        self.list_estimation_p = [0 for i in range(self.shown_size)]
        ## roll
        plt.subplot(2, 1, 1)
        # plt.title("roll")
        plt.xlabel("time[s]")
        plt.ylabel("roll[deg]")
        plt.ylim(-self.ylim, self.ylim)
        plt.grid(True)
        self.line_truth_r, = plt.plot(self.list_t, self.list_truth_r, label="GT")   #get line
        self.line_estimation_r, = plt.plot(self.list_t, self.list_estimation_r, label="Est.")   #get line
        plt.legend()
        ## pitch
        plt.subplot(2, 1, 2)
        # plt.title("pitch")
        plt.xlabel("time[s]")
        plt.ylabel("pitch[deg]")
        plt.ylim(-self.ylim, self.ylim)
        plt.grid(True)
        self.line_truth_p, = plt.plot(self.list_t, self.list_truth_p, label="GT")   #get line
        self.line_estimation_p, = plt.plot(self.list_t, self.list_estimation_p, label="Est.")   #get line
        plt.legend()

    def mainLoop(self):
        while not rospy.is_shutdown():
            if self.got_new_msg:
                self.updatePlot()
                self.got_new_msg = False
            self.drawPlot()

    def updatePlot(self):
        ## append
        t = time.time() - self.start_time
        self.list_t.append(t)
        self.list_truth_r.append(self.truth.vector.x/math.pi*180.0)
        self.list_truth_p.append(self.truth.vector.y/math.pi*180.0)
        self.list_estimation_r.append(self.estimation.vector.x/math.pi*180.0)
        self.list_estimation_p.append(self.estimation.vector.y/math.pi*180.0)
        ## pop
        if self.erase_old_data:
            self.list_t.pop(0)
            self.list_truth_r.pop(0)
            self.list_truth_p.pop(0)
            self.list_estimation_r.pop(0)
            self.list_estimation_p.pop(0)
        ## roll
        plt.subplot(2,1,1)
        self.line_truth_r.set_xdata(self.list_t)
        self.line_truth_r.set_ydata(self.list_truth_r)
        self.line_estimation_r.set_xdata(self.list_t)
        self.line_estimation_r.set_ydata(self.list_estimation_r)
        plt.xlim(min(self.list_t), max(self.list_t))
        ## pitch
        plt.subplot(2,1,2)
        self.line_truth_p.set_xdata(self.list_t)
        self.line_truth_p.set_ydata(self.list_truth_p)
        self.line_estimation_p.set_xdata(self.list_t)
        self.line_estimation_p.set_ydata(self.list_estimation_p)
        plt.xlim(min(self.list_t), max(self.list_t))

    def drawPlot(self):
        ## draw
        plt.draw()
        plt.pause(self.interval)

def main():
    rospy.init_node('compare_rpy', anonymous=True)

    compare_rpy = CompareRPY()

    rospy.spin()

if __name__ == '__main__':
    main()
