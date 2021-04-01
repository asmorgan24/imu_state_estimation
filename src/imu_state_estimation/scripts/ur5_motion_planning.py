#! /usr/bin/python
import pickle
import rospy, rospkg, PyKDL, math
import IPython
import sensor_msgs.point_cloud2 as pc2
import tf, time
import ctypes, struct
import numpy as np
from geometry_msgs.msg import PointStamped, Point
import std_msgs
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, PositionConstraint
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import time
from std_msgs.msg import Int32


class ur5PlannerNode():

    def __init__(self):

        self.robot = RobotCommander()
        self.arm_group = MoveGroupCommander("manipulator")
        self.arm_group.set_goal_orientation_tolerance(0.1)
        self.arm_group.set_goal_position_tolerance(0.1)

        # rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            invalid_move = True
            while invalid_move:
                self.arm_group.set_random_target()
                plan = self.arm_group.plan()
                invalid_move = False if len(plan.joint_trajectory.points) > 3 else True
            self.arm_group.execute(plan)
            # rate.sleep()
            time.sleep(10)



if __name__ == '__main__':

    rospy.init_node("ur5Planner")
    ur5PlannerNode()
