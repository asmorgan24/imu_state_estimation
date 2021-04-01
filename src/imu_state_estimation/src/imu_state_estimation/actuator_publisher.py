#! /usr/bin/python

import rospy
import tf2_ros, PyKDL, tf
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import IPython
import numpy as np

class AP():

    def __init__(self):
        self.actuatorPublisher = rospy.Publisher('/actuator_publisher', MarkerArray, queue_size=1)

        self.listener = tf.TransformListener()
        self.actuatorArray = MarkerArray()
        #ORDER:
        # 6 spheres for joints, 6 cylinders for internal primsatic shafts, 6 cylinders for external prismatic housings

        self.external_actuator_length = 0.17


        #Joint Spheres
        for i in range(6):
            marker = self.createJoint([0,0,0.1], [0,0,0], i+1, size =0.06)
            self.actuatorArray.markers.append(marker)
        #Internal actuators
        for i in range(6):
            marker = self.createActuator([0,0,0.1], [0,0,0], i+1+6, roundness =0.03, length=0.2)
            self.actuatorArray.markers.append(marker)
        #External acutator housings
        for i in range(6):
            marker = self.createActuator([0,0,0.1], [0,0,0], i+1+12, roundness = 0.05, length = self.external_actuator_length, color = [1,1,1])
            self.actuatorArray.markers.append(marker)



        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self._adjust_actuators()
            self.actuatorPublisher.publish(self.actuatorArray)
            rate.sleep()


    def _adjust_actuators(self):
        self.listener.waitForTransform("world", "stewart_base_link", rospy.Time(0), rospy.Duration(4.0))
        (trans, rot)  = self.listener.lookupTransform("world", "stewart_base_link", rospy.Time(0))
        base = PyKDL.Frame(PyKDL.Rotation.Quaternion(*rot), PyKDL.Vector(*trans))
        self.listener.waitForTransform("world", "stewart_platform_link", rospy.Time(0), rospy.Duration(4.0))
        (trans, rot)  = self.listener.lookupTransform("world", "stewart_platform_link", rospy.Time(0))
        platform = PyKDL.Frame(PyKDL.Rotation.Quaternion(*rot), PyKDL.Vector(*trans))

        radius = 0.18
        f11 = build_off_frame(radius*np.cos(0),radius*np.sin(0))
        f12 = build_off_frame(radius*np.cos(2*np.pi/3),radius*np.sin(2*np.pi/3))
        f13 = build_off_frame(radius*np.cos(4*np.pi/3),radius*np.sin(4*np.pi/3))

        f21 = build_off_frame(radius*np.cos(np.pi/3),radius*np.sin(np.pi/3))
        f22 = build_off_frame(radius*np.cos(np.pi),radius*np.sin(np.pi))
        f23 = build_off_frame(radius*np.cos(5*np.pi/3),radius*np.sin(5*np.pi/3))

        pairs = [[base*f11,platform*f21],
                 [base*f12,platform*f21],
                 [base*f12,platform*f22],
                 [base*f13,platform*f22],
                 [base*f13,platform*f23],
                 [base*f11,platform*f23]]

        frames = [base*f11,base*f12,base*f13,platform*f21,platform*f22,platform*f23]
        #Update the sphere location
        for i in range(len(frames)):
            self.actuatorArray.markers[i].pose.position.x = frames[i].p[0]
            self.actuatorArray.markers[i].pose.position.y = frames[i].p[1]
            self.actuatorArray.markers[i].pose.position.z = frames[i].p[2]


        #Calculate acutator properties
        for i in range(len(pairs)):
            internal_position, external_position, orientation, length = self._calc_actuator_properties(pairs[i][0],pairs[i][1])
            #internal
            self.actuatorArray.markers[i+6].pose.position.x = internal_position[0]
            self.actuatorArray.markers[i+6].pose.position.y = internal_position[1]
            self.actuatorArray.markers[i+6].pose.position.z = internal_position[2]

            #external
            self.actuatorArray.markers[i+12].pose.position.x = external_position[0]
            self.actuatorArray.markers[i+12].pose.position.y = external_position[1]
            self.actuatorArray.markers[i+12].pose.position.z = external_position[2]

            rotation = PyKDL.Rotation.RPY(*orientation).GetQuaternion()

            #internal
            self.actuatorArray.markers[i+6].pose.orientation.x = rotation[0]
            self.actuatorArray.markers[i+6].pose.orientation.y = rotation[1]
            self.actuatorArray.markers[i+6].pose.orientation.z = rotation[2]
            self.actuatorArray.markers[i+6].pose.orientation.w = rotation[3]

            #external
            self.actuatorArray.markers[i+12].pose.orientation.x = rotation[0]
            self.actuatorArray.markers[i+12].pose.orientation.y = rotation[1]
            self.actuatorArray.markers[i+12].pose.orientation.z = rotation[2]
            self.actuatorArray.markers[i+12].pose.orientation.w = rotation[3]

            self.actuatorArray.markers[i+6].scale.z = length


    def _calc_actuator_properties(self, frame1, frame2):
        loc1 = np.asarray(list(frame1.p))
        loc2 = np.asarray(list(frame2.p))
        length = np.linalg.norm(loc2-loc1)
        midpoint = np.mean(np.asarray([[loc1],[loc2]]), axis =0)
        internal_position = midpoint

        vec = PyKDL.Vector(*loc2-loc1)
        vec/=vec.Norm()

        external_end = np.asarray(list(vec * self.external_actuator_length))
        external_position = np.mean(np.asarray([[0,0,0],external_end]), axis =0)+loc1

        roll = math.asin(-vec[1]);
        pitch = np.arctan2(vec[0], vec[2])

        return internal_position[0], external_position, [roll, pitch, 0], length



    def createActuator(self, position, orientation, id, roundness, length, color = [0,0,0]):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()

        marker.id = id
        marker.ns = "actuator"
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]

        rotation = PyKDL.Rotation.RPY(*orientation).GetQuaternion()

        marker.pose.orientation.x = rotation[0]
        marker.pose.orientation.y = rotation[1]
        marker.pose.orientation.z = rotation[2]
        marker.pose.orientation.w = rotation[3]

        marker.scale.x = roundness
        marker.scale.y = roundness
        marker.scale.z = length

        marker.color.r = color[0]
        marker.color.g = color[0]
        marker.color.b = color[0]
        marker.color.a = 1

        return marker

    def createJoint(self, position, orientation, id, size):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()

        marker.id = id
        marker.ns = "joint"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]

        rotation = PyKDL.Rotation.RPY(*orientation).GetQuaternion()

        marker.pose.orientation.x = rotation[0]
        marker.pose.orientation.y = rotation[1]
        marker.pose.orientation.z = rotation[2]
        marker.pose.orientation.w = rotation[3]

        scale = size
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1

        return marker



def build_off_frame(xoff,yoff):
    return PyKDL.Frame(PyKDL.Rotation.RPY(0,0,0), PyKDL.Vector(xoff, yoff, 0))




if __name__ == '__main__':

    rospy.init_node("actuator_publisher_node")
    AP()
