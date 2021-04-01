#! /usr/bin/python
import rospy, rospkg, math, PyKDL
import numpy as np
import IPython
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros


class stewart_ik():

    def __init__(self):

        self.keybaord_sub = rospy.Subscriber('/keyboard_input_py/raw_input', Int32, self.servoing_callback)
        self.br = tf2_ros.StaticTransformBroadcaster()

        self.stopped = False
        NUM_POINTS = 2000
        radius = np.concatenate([np.linspace(0.0,0.38,NUM_POINTS/2),np.linspace(0.38,0.0,NUM_POINTS/2)])
        roundabout = np.concatenate([np.linspace(0,8*np.pi,NUM_POINTS/2),np.linspace(8*np.pi,0,NUM_POINTS/2)])
        roll_ = radius *np.cos(roundabout)
        pitch_ = radius *np.sin(roundabout)

        self._reset_pos()
        rate = rospy.Rate(30)
        counter = 0
        # IPython.embed()
        while not rospy.is_shutdown():
            self._broadcast_pose([0,0,0.2,roll_[counter%NUM_POINTS], pitch_[counter%NUM_POINTS],0])
            rate.sleep()
            counter +=1


    def _reset_pos(self):
        self.x,self.y,self.z,self.roll,self.pitch,self.yaw = 0,0,0.2,0,0,0

    def _broadcast_pose(self, pose = None):
        t = TransformStamped()
        if pose is None or self.stopped==True:
            pose = [self.x,self.y,self.z,self.roll,self.pitch,self.yaw]
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "stewart_base_link"
        t.child_frame_id = "stewart_platform_link"
        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]
        rot = PyKDL.Rotation.RPY(pose[3],pose[4],pose[5]).GetQuaternion()
        t.transform.rotation.x = rot[0]
        t.transform.rotation.y = rot[1]
        t.transform.rotation.z = rot[2]
        t.transform.rotation.w = rot[3]
        self.br.sendTransform(t)


    #decode the keybaord input
    def servoing_callback(self, req , step = 0.02):
        self.stopped = True
        #a=97, s =115, d =100, w =119, x=120
        #q = 113, e =101
        if req.data == 119:
            self.pitch += step
        elif req.data == 120:
            self.pitch += -step
        elif req.data == 97:
            self.roll += step
        elif req.data == 100:
            self.roll += -step
        elif req.data == 113:
            self.yaw += step/2.
        elif req.data == 101:
            self.yaw += -step/2.


        elif req.data == 115:
            self._reset_pos()




if __name__ == '__main__':

    rospy.init_node("stewart_ik")
    stewart_ik()
