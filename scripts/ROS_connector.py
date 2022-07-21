#!/usr/bin/env python
# coding=utf-8

import rospy
import threading
import time
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String, Float64

class ROS_connector():

    def __init__(self, topicID = 'clik_index'):

        

        self.data = Float64 # message type initialization e.g. self.message = LaserScan()
        self.topicID = topicID
        self.ready = False

    def callback(self,data):
        self.data = data
        self.ready = True

    def ROS_node(self):
        print("\tStarting ROS node for clik_index data, passed topic name is \"", self.topicID, "\" (containing clik_index) ...")
        
        rospy.init_node('ROS_connection_node', anonymous=True, disable_signals=True)
        rospy.Subscriber(self.topicID, Float64, self.callback)
        # e.g.
        #rospy.Subscriber("simulated_scan", LaserScan, self.callback_class)
        rospy.spin()

    def Get_data(self):
        self.ready = False
        return self.data


    def DataAvailable(self):
        return self.ready

    def start(self):
        threading.Thread(target=self.subscriber_lidar_class).start()
        #time.sleep(1)
    

####################################################################################################################
# single node for direct execution

def th_callback():

    print("Trajectory received!")

def subscriber_lidar():
    rospy.init_node('subscriber_node', anonymous=True)

    #DECOMMENTARE SE LIDAR SIMULATO
    rospy.Subscriber('clik_index', Float64, th_callback)

    # e.g.
    #rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber_lidar()