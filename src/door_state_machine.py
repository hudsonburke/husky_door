#!/usr/bin/env python3

from enum import Enum, auto
import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


class States(Enum):
    DETECT = auto()
    CLASSIFY = auto()
    OPEN = auto()
    ENTER = auto()


class DoorStateMachine:
    def __init__(self):
        self.__state = States.DETECT

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def set_state(self, state):
        self.__state = state

    def get_state(self):
        return self.__state

    def navigate(self, x, y, yaw=0.0, frame="map"):  # yaw in degrees
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = frame
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y

        (self.goal.target_pose.pose.orientation.x, self.goal.target_pose.pose.orientation.y,
         self.goal.target_pose.pose.orientation.z, self.goal.target_pose.pose.orientation.w) = quaternion_from_euler(0, 0, yaw)
        # default: no rotation

        self.client.send_goal(self.goal)

        # Wait for server to finish
        wait = self.client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            result = self.client.get_result()
            if result:
                rospy.loginfo("Goal execution done!")
                return True


if __name__ == "__main__":
    try:
        rospy.init_node('door_state_machine', anonymous=True)
        machine = DoorStateMachine()
        while not rospy.is_shutdown():
            if machine.get_state() == States.DETECT:
                # Detect the bounds of the door
                pass
            elif machine.get_state() == States.CLASSIFY:
                # Classify the door handle
                pass
            elif machine.get_state() == States.OPEN:
                # Grab handle and open door
                pass
            elif machine.get_state() == States.ENTER:
                # Set a goal inside the room and drop the inflation radius
                rospy.set_param("inflation_radius", 0.001)
                #
                pass
            else:
                pass
            pass
    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")
