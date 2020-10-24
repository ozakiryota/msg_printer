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
        self.axis_resolution = rospy.get_param("/axis_resolution", 10.0)
        print("self.axis_resolution = ", self.axis_resolution)
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
        self.list_sub = []
        for method_idx in range(self.num_sub):
            sub_estimation = rospy.Subscriber("/estimation" + str(method_idx) + "/rpy", Vector3Stamped, self.callbackEstimation, callback_args=method_idx, queue_size=1)
            self.list_sub.append(sub_estimation)
        ## msg
        self.list_estimation_msg = []
        for _ in range(self.num_sub):
            self.list_estimation_msg.append(Vector3Stamped())
        ## list
        self.list_t = []
        self.list_list_estimation_r = []
        self.list_list_estimation_p = []
        for _ in range(self.num_sub):
            self.list_list_estimation_r.append([])
            self.list_list_estimation_p.append([])
        ## line
        self.list_line_estimation_r = []
        self.list_line_estimation_p = []
        ## time
        self.start_time = time.time()
        ## flag
        self.got_first_msg = False
        self.got_new_msg = False
        ## initialization
        self.initializePlot()
        self.initializeCSV()
        ## loop
        self.mainLoop()

    def callbackEstimation(self, msg, method_idx):
        self.list_estimation_msg[method_idx] = msg
        self.got_new_msg = True
        if not self.got_first_msg:
            self.start_time = time.time()
            self.got_first_msg = True

    def initializePlot(self):
        plt.ion()   #interactive mode on
        plt.figure()
        ## empty
        self.list_t = [0 for _ in range(self.ini_shown_size)]
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
        for method_idx in range(self.num_sub):
            line_estimation_p, = plt.plot(self.list_t, self.list_list_estimation_p[method_idx], label=self.list_method_name[method_idx])   #get line
            self.list_line_estimation_p.append(line_estimation_p)
        plt.legend()

    def initializeCSV(self):
        writer = csv.writer(self.csv_file)
        row = ["time"]
        for method_idx in range(self.num_sub):
            row = row + [self.list_method_name[method_idx] + "_r", self.list_method_name[method_idx] + "_p"]
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
        for method_idx in range(self.num_sub):
            self.list_list_estimation_r[method_idx].append(self.list_estimation_msg[method_idx].vector.x/math.pi*180.0)
            self.list_list_estimation_p[method_idx].append(self.list_estimation_msg[method_idx].vector.y/math.pi*180.0)
        ## pop
        if self.erase_old_data:
            self.list_t.pop(0)
            for method_idx in range(self.num_sub):
                self.list_list_estimation_r[method_idx].pop(0)
                self.list_list_estimation_p[method_idx].pop(0)
        ## roll
        plt.subplot(2,1,1)
        ## roll-data
        for method_idx in range(self.num_sub):
            self.list_line_estimation_r[method_idx].set_xdata(self.list_t)
            self.list_line_estimation_r[method_idx].set_ydata(self.list_list_estimation_r[method_idx])
        ## roll-axis
        plt.xlim(min(self.list_t), max(self.list_t))
        if not self.fix_ylim:
            all_r = []
            for method_idx in range(self.num_sub):
                all_r= all_r + self.list_list_estimation_r[method_idx]
            plt.ylim(min(all_r), max(all_r))
        ## roll-title
        title_r = "NowR[deg]:"
        for method_idx in range(self.num_sub):
            title_r = title_r + " " + self.list_method_name[method_idx] \
                + "{:.3f}".format(self.list_list_estimation_r[method_idx][-1])
        plt.title(title_r)
        ## pitch
        plt.subplot(2,1,2)
        ## pitch-data
        for method_idx in range(self.num_sub):
            self.list_line_estimation_p[method_idx].set_xdata(self.list_t)
            self.list_line_estimation_p[method_idx].set_ydata(self.list_list_estimation_p[method_idx])
        ## pitch-axis
        plt.xlim(min(self.list_t), max(self.list_t))
        if not self.fix_ylim:
            all_p = []
            for method_idx in range(self.num_sub):
                all_p = all_p + self.list_list_estimation_p[method_idx]
            plt.ylim(min(all_p), max(all_p))
        ## pitch-title
        title_p = "NowP[deg]:"
        for method_idx in range(self.num_sub):
            title_p = title_p + " " + self.list_method_name[method_idx] \
                + "{:.3f}".format(self.list_list_estimation_p[method_idx][-1])
        plt.title(title_p)

    def writeCSV(self):
        writer = csv.writer(self.csv_file)
        row = [self.list_t[-1]]
        for method_idx in range(self.num_sub):
            row = row + [self.list_list_estimation_r[method_idx][-1], self.list_list_estimation_p[method_idx][-1]]  #rp
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
