#!/usr/bin/env python3
# coding=utf-8

import rospy
import threading
import time
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String, Float64

class ROS_connector():

    def __init__(self, position_topic, velocity_topic, actual_feet_topic):

        self.data_position = Float64MultiArray()
        self.data_velocity = Float64MultiArray()
        self.position_topic = position_topic
        self.velocity_topic = velocity_topic
        self.actual_feet_topic = actual_feet_topic
        self.actual_feet_data = Float64MultiArray()
        self.actual_feet_data.layout.dim = [MultiArrayDimension()]
        self.actual_feet_data.layout.dim[0].stride = 18
        self.ready_position = False

        rospy.init_node('ROS_connection_node', anonymous=True, disable_signals=True)
        rospy.Subscriber(self.position_topic, Float64MultiArray, self.position_callback)
        rospy.Subscriber(self.velocity_topic, Float64MultiArray, self.velocity_callback)
        self.actual_feet_pub = rospy.Publisher('actual_feet', Float64MultiArray, queue_size=1)


    def position_callback(self,data):
        
        self.data_position = data
        self.ready_position = True

    def velocity_callback(self,data):
        self.data_velocity = data



    def ROS_node(self):
        print("\tStarting ROS node.")
        rospy.spin()

    def Get_data(self):
        
        return self.data_position.data, self.data_velocity.data


    def Set_data(self,data):

        self.actual_feet_data.data = data
        self.actual_feet_pub.publish(self.actual_feet_data)
        self.ready_position = False

    def DataAvailable(self):
        return self.ready_position

    def start_thread(self):
        print("Starting thread")
        threading.Thread(target=self.ROS_node).start()


def ROS_connector_main():

    print("ROS_connector is meant to be a library, not for its use on its own!")


if __name__ == '__main__':
    ROS_connector_main()