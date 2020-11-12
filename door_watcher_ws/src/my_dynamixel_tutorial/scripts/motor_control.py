#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int8, Bool
from dynamixel_msgs.msg import MotorStateList

class MotorControl:
    """
    Rotate the motor depending the door number chose by the user
    Each door has a specific position. In this script, we assume there are 2 doors. But we could add more easily.
    """
    DOOR_1_POSITION = 1.3
    DOOR_2_POSITION = 4.0

    def __init__(self):
        self.door_number = 1 # Default door number 
        self.is_moving = False # Does the motor move?
        
        rospy.init_node('motor_control', anonymous=True)
        
        # Create Subscribers and Publishers
        rospy.Subscriber('/door_number', Int8, self.door_number_callback)
        self.pub_rotation = rospy.Publisher('/pan_controller/command', Float64, latch=True, queue_size=1) 
        rospy.Subscriber('/motor_states/pan_tilt_port', MotorStateList, self.motor_states_callback)
        self.pub_is_moving = rospy.Publisher('/is_moving', Bool, queue_size=1) 

        rospy.loginfo("[motor_control] node started...")

    def door_number_callback(self, data):
        """
        Get the door number asked by the user
        """
        self.door_number = data.data

        msg = Float64()
        
        if self.door_number == 1:
            msg.data = self.DOOR_1_POSITION

        elif self.door_number == 2:
            msg.data = self.DOOR_2_POSITION

        #rospy.loginfo("Door number: {}\tPosition: {}".format(self.door_number, msg.data))
        self.pub_rotation.publish(msg)

    def motor_states_callback(self, data):
        """
        Publish to /is_moving if the motor is moving or not
        """
        for element in data.motor_states:
            self.moving = int(element.moving)

        msg = Bool()
        msg.data = self.moving 
        self.pub_is_moving.publish(msg)


if __name__ == '__main__':
    try:
        MotorControl()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
