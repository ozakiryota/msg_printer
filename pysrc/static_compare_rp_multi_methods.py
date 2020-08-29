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
        ## parameter
        self.num_sub = rospy.get_param("/num_sub", 1)
        print("self.num_sub = ", self.num_sub)
        self.list_method_name = []
        for method_idx in range(self.num_sub):
            method_name = rospy.get_param("/method" + str(method_idx), "method" + str(method_idx))
            self.list_method_name.append(method_name)
        print("self.list_method_name = ", self.list_method_name)
        self.ylim = 45.0
        ## subscriber
        self.sub_truth = rospy.Subscriber("/truth/rpy", Vector3Stamped, self.callbackTruth, queue_size=1)
        self.list_sub = []
        for method_idx in range(self.num_sub):
            sub_estimation = rospy.Subscriber("/estimation" + str(method_idx) + "/rpy", Vector3Stamped, self.callbackEstimation, callback_args=method_idx, queue_size=1)
            self.list_sub.append(sub_estimation)
        ## publisher
        # self.list_pub = []
        # for method_idx in range(self.num_sub):
        #     pub_error = rospy.Publisher("/method" + str(method_idx) + "/error/rpy", Vector3Stamped, queue_size=1)
        #     self.list_pub.append(pub_error)
        ## msg
        self.truth_msg = Vector3Stamped()
        self.list_estimation_msg = []
        self.list_error_msg = []
        for _ in range(self.num_sub):
            self.list_estimation_msg.append(Vector3Stamped())
            self.list_error_msg.append(Vector3Stamped())
        ## list
        self.list_truth_t = []
        self.list_truth_r = []
        self.list_truth_p = []
        self.list_list_estimation_t = []
        self.list_list_estimation_r = []
        self.list_list_estimation_p = []
        self.list_list_error_rp = []
        for _ in range(self.num_sub):
            self.list_list_estimation_t.append([])
            self.list_list_estimation_r.append([])
            self.list_list_estimation_p.append([])
            self.list_list_error_rp.append([])
        ## time
        self.start_time = time.time()

    def callbackTruth(self, msg):
        self.truth_msg = msg
        self.appendData(msg, self.list_truth_t, self.list_truth_r, self.list_truth_p)

    def callbackEstimation(self, msg, method_idx):
        self.list_estimation_msg[method_idx] = msg
        self.appendData(msg, self.list_list_estimation_t[method_idx], self.list_list_estimation_r[method_idx], self.list_list_estimation_p[method_idx])
        self.computeError(method_idx)

    def appendData(self, msg, list_t, list_r, list_p):
        t = time.time() - self.start_time
        list_t.append(t)
        list_r.append(msg.vector.x/math.pi*180.0)
        list_p.append(msg.vector.y/math.pi*180.0)
    
    def computeError(self, method_idx):
        self.list_error_msg[method_idx].vector.x = self.computeAngleDiff(
            self.truth_msg.vector.x,
            self.list_estimation_msg[method_idx].vector.x
        )
        self.list_error_msg[method_idx].vector.y = self.computeAngleDiff(
            self.truth_msg.vector.y,
            self.list_estimation_msg[method_idx].vector.y
        )
        self.list_error_msg[method_idx].vector.z = self.computeAngleDiff(
            self.truth_msg.vector.z,
            self.list_estimation_msg[method_idx].vector.z
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

    def showPlot(self):
        ## initialize
        plt.figure()
        ## roll
        plt.subplot(2, 1, 1)
        plt.xlabel("time[s]")
        plt.ylabel("roll[deg]")
        plt.xlim(min(self.list_truth_t), max(self.list_truth_t))
        plt.ylim(-self.ylim, self.ylim)
        plt.grid(True)
        plt.plot(self.list_truth_t, self.list_truth_r, label="Truth")
        for method_idx in range(self.num_sub):
            plt.plot(self.list_list_estimation_t[method_idx], self.list_list_estimation_r[method_idx], label=self.list_method_name[method_idx])
        plt.legend()
        title_r = "MAE[deg]: "
        for method_idx in range(self.num_sub):
            title_r = title_r + self.list_method_name[method_idx] + " {:.3f} ".format(self.computeMAE(np.array(self.list_list_error_rp[method_idx]))[0]/math.pi*180.0)
        plt.title(title_r)
        ## pitch
        plt.subplot(2, 1, 2)
        plt.xlabel("time[s]")
        plt.ylabel("pitch[deg]")
        plt.xlim(min(self.list_truth_t), max(self.list_truth_t))
        plt.ylim(-self.ylim, self.ylim)
        plt.grid(True)
        plt.plot(self.list_truth_t, self.list_truth_p, label="Truth")
        for method_idx in range(self.num_sub):
            plt.plot(self.list_list_estimation_t[method_idx], self.list_list_estimation_p[method_idx], label=self.list_method_name[method_idx])
        plt.legend()
        title_p = "MAE[deg]: "
        for method_idx in range(self.num_sub):
            title_p = title_p + self.list_method_name[method_idx] + " {:.3f} ".format(self.computeMAE(np.array(self.list_list_error_rp[method_idx]))[1]/math.pi*180.0)
        plt.title(title_p)
        ## layout
        plt.tight_layout()
        ## show
        plt.show()

    def computeMAE(self, x):
        return np.mean(np.abs(x), axis=0)

    # def publication(self):
    #     for method_idx in range(self.num_sub):
    #         self.list_error_msg[method_idx].header = self.list_estimation_msg[method_idx].header
    #         self.list_pub[method_idx].publish(self.list_error_msg[method_idx])

def main():
    rospy.init_node('compare_rpy', anonymous=True)

    compare_rpy = CompareRPY()

    rospy.spin()
    compare_rpy.showPlot()

if __name__ == '__main__':
    main()
