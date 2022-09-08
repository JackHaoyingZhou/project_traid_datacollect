import os
import sys
import numpy as np
import rospy
from joint_pos_recorder import JointPosRecorder
from geometry_msgs.msg import TransformStamped
import time

jpRecorder_sujecm = JointPosRecorder(save_path='./data/SUJECM', record_size=100)
jpRecorder_sujpsm2 = JointPosRecorder(save_path='./data/SUJPSM2', record_size=100)
jpRecorder_ecm = JointPosRecorder(save_path='./data/ECM', record_size=100)

class triadRecorder:
    def __init__(self):
        rospy.init_node('test', anonymous=True)
        self.sujecm_topic = '/SUJ/ECM/measured_cp'
        self.sujpsm2_topic = '/SUJ/PSM2/measured_cp'
        self.ecm_topic = '/ECM/measured_cp'
        self.list_sujecm = None
        self.list_ecm = None
        self.list_sujpsm2 = None
        self.sub_topic_sujecm = rospy.Subscriber(self.sujecm_topic, TransformStamped, self.sujecm_sub, queue_size=1)
        self.sub_topic_sujpsm2 = rospy.Subscriber(self.sujpsm2_topic, TransformStamped, self.sujpsm2_sub, queue_size=1)
        self.sub_topic_ecm = rospy.Subscriber(self.ecm_topic, TransformStamped, self.ecm_sub, queue_size=1)

    def sujecm_sub(self, msg):
        tranform_cp = msg.transform
        sx = tranform_cp.translation.x
        sy = tranform_cp.translation.y
        sz = tranform_cp.translation.z
        x = tranform_cp.rotation.x
        y = tranform_cp.rotation.y
        z = tranform_cp.rotation.z
        w = tranform_cp.rotation.w
        self.list_sujecm = [sx, sy, sz, x, y, z, w]

    def ecm_sub(self, msg):
        tranform_cp = msg.transform
        sx = tranform_cp.translation.x
        sy = tranform_cp.translation.y
        sz = tranform_cp.translation.z
        x = tranform_cp.rotation.x
        y = tranform_cp.rotation.y
        z = tranform_cp.rotation.z
        w = tranform_cp.rotation.w
        self.list_ecm = [sx, sy, sz, x, y, z, w]

    def sujpsm2_sub(self, msg):
        tranform_cp = msg.transform
        sx = tranform_cp.translation.x
        sy = tranform_cp.translation.y
        sz = tranform_cp.translation.z
        x = tranform_cp.rotation.x
        y = tranform_cp.rotation.y
        z = tranform_cp.rotation.z
        w = tranform_cp.rotation.w
        self.list_sujpsm2 = [sx, sy, sz, x, y, z, w]

    def get_data(self):
        return self.list_sujecm, self.list_sujpsm2, self.list_ecm

    def sub_run(self, feq):
        rate = rospy.Rate(feq)
        count = 0
        while (count< 10):
            list_sujecm, list_sujpsm2, list_ecm = self.get_data()
            # print(list_sujecm)
            jpRecorder_sujecm.record(list_sujecm)
            # print(list_sujpsm2)
            jpRecorder_sujpsm2.record(list_sujpsm2)
            jpRecorder_ecm.record(list_ecm)
            rate.sleep()
            count = count + 1
        jpRecorder_sujecm.flush()
        jpRecorder_sujpsm2.flush()
        jpRecorder_ecm.flush()

if __name__ == '__main__':
    sub_class = triadRecorder()
    sub_class.sub_run(1)


