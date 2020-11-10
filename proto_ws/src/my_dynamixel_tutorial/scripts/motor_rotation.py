#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float64, Int8
from dynamixel_msgs.msg import MotorStateList


class MotorRotation:
    """
    Rotate the motor depending the door number chose by the user
    Each door has a specific position. In this script, we assume there are 2 doors. But we could add more easily.
    """
    DOOR_1_POSITION = 1.3
    DOOR_2_POSITION = 4.0

    def __init__(self):
        self.door_number = 1 # Default door number 
        self.current_goal_position = 0 # Position 
        
        rospy.init_node('motor_rotation', anonymous=True)
        
        # Create Subscribers and Publishers
        rospy.Subscriber('/door_number', Int8, self.door_number_callback)
        rospy.Subscriber('/motor_states/pan_tilt_port', MotorStateList, self.current_position_callback)
        self.pub = rospy.Publisher('/pan_controller/command', Float64, latch=True, queue_size=1)
        
        rospy.loginfo("[motor_rotation] node started...")

    def door_number_callback(self, data):
        """
        Get the door number asked by the user
        """
        self.door_number = data.data

    def current_position_callback(self, data):
        """
        Get the current goal position 
        """
        for element in data.motor_states:
            self.current_goal_position = int(element.goal)

    def publish_position(self):
        """
        Rotate the robot to the door number asked by the user
        """
        msg = Float64()
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            
            rospy.wait_for_message('/door_number', MotorStateList, timeout=None)
            rospy.wait_for_message('/motor_states/pan_tilt_port', MotorStateList, timeout=None)

            if self.door_number == 1:
                msg.data = self.DOOR_1_POSITION
                if self.current_goal_position == 254:
                    rospy.logwarn("Already on door 1")

            elif self.door_number == 2:
                msg.data = self.DOOR_2_POSITION
                if self.current_goal_position == 782:
                    rospy.logwarn("Already on door 2")

            rospy.loginfo("Door number: {}\tPosition: {}".format(self.door_number, msg.data))
            self.pub.publish(msg)
            rate.sleep()
    

if __name__ == '__main__':
    try:
        obj = MotorRotation()
        obj.publish_position()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
