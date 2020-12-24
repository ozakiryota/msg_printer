#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

import math
import time
import matplotlib.pyplot as plt
import numpy as np

import os
import csv

class RecordFloatMsg:
    def __init__(self):
        print("--- record_float_msg ---")
        ## parameter-graph
        self.erase_old_data = rospy.get_param("/erase_old_data", True)
        print("self.erase_old_data = ", self.erase_old_data)
        self.ini_shown_size = rospy.get_param("/ini_shown_size", 100)
        if not self.erase_old_data:
            self.ini_shown_size = 1
        print("self.ini_shown_size = ", self.ini_shown_size)
        self.fix_ylim = rospy.get_param("/fix_ylim", False)
        print("self.fix_ylim = ", self.fix_ylim)
        self.ylim = rospy.get_param("/ylim", 0.0)
        print("self.ylim = ", self.ylim)
        self.interval = rospy.get_param("/interval", 0.1)
        print("self.interval = ", self.interval)
        ## parameter-csv
        self.save_csv = rospy.get_param("/save_csv", True)
        print("self.save_csv = ", self.save_csv)
        if self.save_csv:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            csv_name = "record_float_msg"
            csv_path = rospy.get_param("/csv_path", current_dir + "/../save/" + csv_name + ".csv")
            self.csv_file = open(csv_path, "w")
            print("csv_path = ", csv_path)
        ## subscriber
        sub_float = rospy.Subscriber("/float", Float64, self.callbackFloat, queue_size=1)
        ## msg
        self.float_msg = Float64
        ## list
        self.list_t = []
        self.list_data = []
        ## line
        self.line = None
        ## time
        self.start_time = time.time()
        ## flag
        self.got_first_msg = False
        self.got_new_msg = False
        ## initialization
        self.initializePlot()
        if self.save_csv:
            self.initializeCSV()
        ## loop
        self.mainLoop()

    def callbackFloat(self, msg):
        self.float_msg = msg
        self.got_new_msg = True
        if not self.got_first_msg:
            self.start_time = time.time()
            self.got_first_msg = True

    def initializePlot(self):
        plt.ion()   #interactive mode on
        plt.figure()
        ## empty
        self.list_t = [0 for _ in range(self.ini_shown_size)]
        self.list_data = [0 for _ in range(self.ini_shown_size)]
        # plt
        plt.xlabel("time[s]")
        plt.ylabel("data[?]")
        plt.ylim(-self.ylim, self.ylim)
        plt.grid(True)
        self.line, = plt.plot(self.list_t, self.list_data)   #get line

    def initializeCSV(self):
        writer = csv.writer(self.csv_file)
        row = ["time", "data"]
        writer.writerow(row)

    def mainLoop(self):
        while not rospy.is_shutdown():
            if self.got_new_msg:
                self.updatePlot()
                if self.save_csv:
                    self.writeCSV()
                self.got_new_msg = False
            self.drawPlot()

    def updatePlot(self):
        ## append
        t = time.time() - self.start_time
        self.list_t.append(t)
        self.list_data.append(self.float_msg.data)
        ## pop
        if self.erase_old_data:
            self.list_t.pop(0)
            self.list_data.pop(0)
        ## set
        self.line.set_xdata(self.list_t)
        self.line.set_ydata(self.list_data)
        ## axis
        plt.xlim(min(self.list_t), max(self.list_t))
        if not self.fix_ylim:
            plt.ylim(min(self.list_data), max(self.list_data))
        ## title
        title = "data[?] = " + "{:.3f}".format(self.list_data[-1])
        plt.title(title)

    def writeCSV(self):
        writer = csv.writer(self.csv_file)
        row = [self.list_t[-1], self.list_data[-1]]
        writer.writerow(row)

    def drawPlot(self):
        ## layout
        plt.tight_layout()
        ## draw
        plt.draw()
        plt.pause(self.interval)

def main():
    rospy.init_node('record_float_msg', anonymous=True)

    record_float_msg = RecordFloatMsg()

    rospy.spin()

if __name__ == '__main__':
    main()
