#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped

import math
import time
import matplotlib.pyplot as plt
import numpy as np

import os
import csv

class CompareRPY:
    def __init__(self):
        print("--- compare_rpy ---")
        ## parameter-graph
        self.erase_old_data = rospy.get_param("/erase_old_data", True)
        print("self.erase_old_data = ", self.erase_old_data)
        self.ini_shown_size = rospy.get_param("/ini_shown_size", 100)
        if not self.erase_old_data:
            self.ini_shown_size = 1
        print("self.ini_shown_size = ", self.ini_shown_size)
        self.fix_ylim = rospy.get_param("/fix_ylim", False)
        print("self.fix_ylim = ", self.fix_ylim)
        self.ylim = rospy.get_param("/ylim", 45.0)
        print("self.ylim = ", self.ylim)
        self.interval = rospy.get_param("/interval", 0.1)
        print("self.interval = ", self.interval)
        ## parameter-method
        self.num_sub = rospy.get_param("/num_sub", 1)
        print("self.num_sub = ", self.num_sub)
        self.list_method_name = []
        for method_idx in range(self.num_sub):
            method_name = rospy.get_param("/method" + str(method_idx), "method" + str(method_idx))
            self.list_method_name.append(method_name)
        print("self.list_method_name = ", self.list_method_name)
        ## parameter-csv
        current_dir = os.path.dirname(os.path.abspath(__file__))
        csv_name = "rp"
        for method_idx in range(self.num_sub):
            csv_name = csv_name + "_" + self.list_method_name[method_idx]
        csv_path = rospy.get_param("/csv_path", current_dir + "/../save/" + csv_name + ".csv")
        self.csv_file = open(csv_path, "w")
        print("csv_path = ", csv_path)
        ## subscriber
        self.sub_truth = rospy.Subscriber("/truth/rpy", Vector3Stamped, self.callbackTruth, queue_size=1)
        self.list_sub = []
        for method_idx in range(self.num_sub):
            sub_estimation = rospy.Subscriber("/estimation" + str(method_idx) + "/rpy", Vector3Stamped, self.callbackEstimation, callback_args=method_idx, queue_size=1)
            self.list_sub.append(sub_estimation)
        ## publisher
        self.list_pub = []
        for method_idx in range(self.num_sub):
            pub_error = rospy.Publisher("/method" + str(method_idx) + "/error/rpy", Vector3Stamped, queue_size=1)
            self.list_pub.append(pub_error)
        ## msg
        self.truth_msg = Vector3Stamped()
        self.list_estimation_msg = []
        self.list_error_msg = []
        for _ in range(self.num_sub):
            self.list_estimation_msg.append(Vector3Stamped())
            self.list_error_msg.append(Vector3Stamped())
        ## list
        self.list_t = []
        self.list_truth_r = []
        self.list_truth_p = []
        self.list_list_estimation_r = []
        self.list_list_estimation_p = []
        self.list_list_error_rp = []
        for _ in range(self.num_sub):
            self.list_list_estimation_r.append([])
            self.list_list_estimation_p.append([])
            self.list_list_error_rp.append([])
        ## line
        self.line_truth_r = None
        self.line_truth_p = None
        self.list_line_estimation_r = []
        self.list_line_estimation_p = []
        ## time
        self.start_time = time.time()
        ## flag
        self.got_first_truth = False
        self.got_new_msg = False

        ## initialization
        self.initializePlot()
        self.initializeCSV()
        ## loop
        self.mainLoop()

    def callbackTruth(self, msg):
        self.truth_msg = msg
        if not self.got_first_truth:
            self.start_time = time.time()
            self.got_first_truth = True

    def callbackEstimation(self, msg, method_idx):
        self.list_estimation_msg[method_idx] = msg
        if self.got_first_truth:
            self.computeError(method_idx)
            self.publication()
            self.got_new_msg = True
    
    def computeError(self, method_idx):
        self.list_error_msg[method_idx].vector.x = self.computeAngleDiff(
            self.list_estimation_msg[method_idx].vector.x,
            self.truth_msg.vector.x
        )
        self.list_error_msg[method_idx].vector.y = self.computeAngleDiff(
            self.list_estimation_msg[method_idx].vector.y,
            self.truth_msg.vector.y
        )
        self.list_error_msg[method_idx].vector.z = self.computeAngleDiff(
            self.list_estimation_msg[method_idx].vector.z,
            self.truth_msg.vector.z
        )
        ## append
        self.list_list_error_rp[method_idx].append([self.list_error_msg[method_idx].vector.x, self.list_error_msg[method_idx].vector.y])
        ## print
        # print("---", self.list_method_name[method_idx], "---")
        # arr_error_rp = np.array(self.list_list_error_rp[method_idx])
        # print("arr_error_rp.shape = ", arr_error_rp.shape)
        # print("MAE[deg]: ", self.computeMAE(arr_error_rp)/math.pi*180.0)
    
    def computeAngleDiff(self, x, y):
        diff = math.atan2(math.sin(x - y), math.cos(x - y))
        return diff

    def computeMAE(self, x):
        return np.mean(np.abs(x), axis=0)

    def publication(self):
        for method_idx in range(self.num_sub):
            self.list_error_msg[method_idx].header = self.list_estimation_msg[method_idx].header
            self.list_pub[method_idx].publish(self.list_error_msg[method_idx])

    def initializePlot(self):
        plt.ion()   #interactive mode on
        plt.figure()
        ## empty
        self.list_t = [0 for _ in range(self.ini_shown_size)]
        self.list_truth_r = [0 for _ in range(self.ini_shown_size)]
        self.list_truth_p = [0 for _ in range(self.ini_shown_size)]
        for method_idx in range(self.num_sub):
            self.list_list_estimation_r[method_idx] = [0 for _ in range(self.ini_shown_size)]
            self.list_list_estimation_p[method_idx] = [0 for _ in range(self.ini_shown_size)]
        ## roll
        plt.subplot(2, 1, 1)
        # plt.title("roll")
        plt.xlabel("time[s]")
        plt.ylabel("roll[deg]")
        plt.ylim(-self.ylim, self.ylim)
        plt.grid(True)
        self.line_truth_r, = plt.plot(self.list_t, self.list_truth_r, label="Truth")   #get line
        for method_idx in range(self.num_sub):
            line_estimation_r, = plt.plot(self.list_t, self.list_list_estimation_r[method_idx], label=self.list_method_name[method_idx])   #get line
            self.list_line_estimation_r.append(line_estimation_r)
        plt.legend()
        ## pitch
        plt.subplot(2, 1, 2)
        # plt.title("pitch")
        plt.xlabel("time[s]")
        plt.ylabel("pitch[deg]")
        plt.ylim(-self.ylim, self.ylim)
        plt.grid(True)
        self.line_truth_p, = plt.plot(self.list_t, self.list_truth_p, label="Truth")   #get line
        for method_idx in range(self.num_sub):
            line_estimation_p, = plt.plot(self.list_t, self.list_list_estimation_p[method_idx], label=self.list_method_name[method_idx])   #get line
            self.list_line_estimation_p.append(line_estimation_p)
        plt.legend()

    def initializeCSV(self):
        writer = csv.writer(self.csv_file)
        row = ["time", "Truth_r", "Truth_p"]
        for method_idx in range(self.num_sub):
            row = row + [self.list_method_name[method_idx] + "_r", self.list_method_name[method_idx] + "_p"]
            row = row + ["E_" + self.list_method_name[method_idx] + "_r", "E_" + self.list_method_name[method_idx] + "_p"]
        writer.writerow(row)

    def mainLoop(self):
        while not rospy.is_shutdown():
            if self.got_new_msg:
                self.updatePlot()
                self.writeCSV()
                self.got_new_msg = False
            self.drawPlot()

    def updatePlot(self):
        ## append
        t = time.time() - self.start_time
        self.list_t.append(t)
        self.list_truth_r.append(self.truth_msg.vector.x/math.pi*180.0)
        self.list_truth_p.append(self.truth_msg.vector.y/math.pi*180.0)
        for method_idx in range(self.num_sub):
            self.list_list_estimation_r[method_idx].append(self.list_estimation_msg[method_idx].vector.x/math.pi*180.0)
            self.list_list_estimation_p[method_idx].append(self.list_estimation_msg[method_idx].vector.y/math.pi*180.0)
        ## pop
        if self.erase_old_data:
            self.list_t.pop(0)
            self.list_truth_r.pop(0)
            self.list_truth_p.pop(0)
            for method_idx in range(self.num_sub):
                self.list_list_estimation_r[method_idx].pop(0)
                self.list_list_estimation_p[method_idx].pop(0)
        ## roll
        plt.subplot(2,1,1)
        ## roll-data
        self.line_truth_r.set_xdata(self.list_t)
        self.line_truth_r.set_ydata(self.list_truth_r)
        for method_idx in range(self.num_sub):
            self.list_line_estimation_r[method_idx].set_xdata(self.list_t)
            self.list_line_estimation_r[method_idx].set_ydata(self.list_list_estimation_r[method_idx])
        ## roll-axis
        plt.xlim(min(self.list_t), max(self.list_t))
        if not self.fix_ylim:
            all_r = self.list_truth_r
            for method_idx in range(self.num_sub):
                all_r= all_r + self.list_list_estimation_r[method_idx]
            plt.ylim(min(all_r), max(all_r))
        ## roll-title
        title_r = "MAE|NowErr[deg]:"
        for method_idx in range(self.num_sub):
            title_r = title_r + " " + self.list_method_name[method_idx] \
                + "{:.3f}".format(self.computeMAE(np.array(self.list_list_error_rp[method_idx]))[0]/math.pi*180.0) \
                + "|{:.3f}".format(self.list_list_error_rp[method_idx][-1][0]/math.pi*180.0)
        plt.title(title_r)
        ## pitch
        plt.subplot(2,1,2)
        ## pitch-data
        self.line_truth_p.set_xdata(self.list_t)
        self.line_truth_p.set_ydata(self.list_truth_p)
        for method_idx in range(self.num_sub):
            self.list_line_estimation_p[method_idx].set_xdata(self.list_t)
            self.list_line_estimation_p[method_idx].set_ydata(self.list_list_estimation_p[method_idx])
        ## pitch-axis
        plt.xlim(min(self.list_t), max(self.list_t))
        if not self.fix_ylim:
            all_p = self.list_truth_p
            for method_idx in range(self.num_sub):
                all_p = all_p + self.list_list_estimation_p[method_idx]
            plt.ylim(min(all_p), max(all_p))
        ## pitch-title
        title_p = "MAE|NowErr[deg]:"
        for method_idx in range(self.num_sub):
            title_p = title_p + " " + self.list_method_name[method_idx] \
                + "{:.3f}".format(self.computeMAE(np.array(self.list_list_error_rp[method_idx]))[1]/math.pi*180.0) \
                + "|{:.3f}".format(self.list_list_error_rp[method_idx][-1][1]/math.pi*180.0)
        plt.title(title_p)

    def writeCSV(self):
        writer = csv.writer(self.csv_file)
        row = [self.list_t[-1], self.list_truth_r[-1], self.list_truth_p[-1]]
        for method_idx in range(self.num_sub):
            row = row + [self.list_list_estimation_r[method_idx][-1], self.list_list_estimation_p[method_idx][-1]]  #rp
            row = row + [self.list_list_error_rp[method_idx][-1][0], self.list_list_error_rp[method_idx][-1][1]]  #error_rp
        writer.writerow(row)

    def drawPlot(self):
        ## layout
        plt.tight_layout()
        ## draw
        plt.draw()
        plt.pause(self.interval)

def main():
    rospy.init_node('compare_rpy', anonymous=True)

    compare_rpy = CompareRPY()

    rospy.spin()

if __name__ == '__main__':
    main()
