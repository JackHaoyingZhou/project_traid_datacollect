import os
import sys
import copy
import numpy as np
import rospy
# from joint_pos_recorder import JointPosRecorder
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from TraidPSMFK import TraidPSMFK
from scipy.spatial.transform import Rotation as Rot
import time
from datetime import datetime
import json

dynamic_path = os.path.abspath(__file__+"/../")
# print(dynamic_path)
sys.path.append(dynamic_path)

tm_format = '%Y_%m_%d_%H_%M_%S'

time_current = datetime.now().strftime(tm_format)

save_folder_name = 'exp_' + time_current

save_folder = os.path.join(dynamic_path, 'data_230126_test', save_folder_name)

transform_folder = os.path.join(save_folder, 'transform')

if not os.path.exists(transform_folder):
    os.makedirs(transform_folder)

# jpRecorder = JointPosRecorder(save_path=transform_folder, record_size=50)

img_folder = os.path.join(save_folder, 'image')

if not os.path.exists(img_folder):
    os.makedirs(img_folder)

bridge = CvBridge()

class triadRecorder:
    def __init__(self):
        rospy.init_node('triad_test', anonymous=True)
        self.sujecm_topic = '/SUJ/ECM/measured_cp'
        self.sujpsm2_topic = '/SUJ/PSM2/measured_cp'
        self.psm2_topic = '/PSM2/measured_js'
        self.ecm_topic = '/ECM/measured_cp'
        self.camera1_topic = '/cv_camera_left_0/image_raw'
        self.camera2_topic = '/cv_camera_right_2/image_raw'
        self.robot_topic = '/robot_ready_flag'
        self.pa_topic = '/PA_ready_flag'
        self.record_topic = '/record_ready_flag'
        init_mtx = np.eye(4)
        self.T_sujecm = init_mtx
        self.T_ecm = init_mtx
        self.T_sujpsm2 = init_mtx
        self.T_psm2 = init_mtx
        self.camera1_img = None
        self.camera2_img = None
        self.robot_status = False
        self.pa_status = False
        self.record_status = False
        self.sub_topic_sujecm = rospy.Subscriber(self.sujecm_topic, TransformStamped, self.sujecm_sub, queue_size=1)
        self.sub_topic_sujpsm2 = rospy.Subscriber(self.sujpsm2_topic, TransformStamped, self.sujpsm2_sub, queue_size=1)
        self.sub_topic_ecm = rospy.Subscriber(self.ecm_topic, TransformStamped, self.ecm_sub, queue_size=1)
        self.sub_topic_psm2 = rospy.Subscriber(self.psm2_topic, JointState, self.psm2_sub, queue_size=1)
        self.sub_topic_camera1 = rospy.Subscriber(self.camera1_topic, Image, self.camera1_sub, queue_size=1)
        self.sub_topic_camera2 = rospy.Subscriber(self.camera2_topic, Image, self.camera2_sub, queue_size=1)
        self.sub_robot = rospy.Subscriber(self.robot_topic, Bool, self.robot_sub, queue_size=1)
        self.sub_pa = rospy.Subscriber(self.pa_topic, Bool, self.pa_sub, queue_size=1)
        self.sub_record = rospy.Subscriber(self.record_topic, Bool, self.record_sub, queue_size=1)
        self.count = 0

    def sujecm_sub(self, msg):
        tranform_cp = msg.transform
        sx = tranform_cp.translation.x
        sy = tranform_cp.translation.y
        sz = tranform_cp.translation.z
        x = tranform_cp.rotation.x
        y = tranform_cp.rotation.y
        z = tranform_cp.rotation.z
        w = tranform_cp.rotation.w
        T = np.zeros((4, 4))
        r = Rot.from_quat([x, y, z, w])
        T[3, 3] = 1
        T[0:3, 0:3] = r.as_matrix()
        T[0, 3] = sx
        T[1, 3] = sy
        T[2, 3] = sz
        self.T_sujecm = T

    def psm2_sub(self, msg):
        joint_pos = msg.position
        q = np.zeros((1, 3))
        q[0, 0] = joint_pos[0]
        q[0, 1] = joint_pos[1]
        q[0, 2] = joint_pos[2] - 0.20
        PSM_FK = TraidPSMFK(q)
        T = PSM_FK.compute_FK()
        self.T_psm2 = T

    def ecm_sub(self, msg):
        tranform_cp = msg.transform
        sx = tranform_cp.translation.x
        sy = tranform_cp.translation.y
        sz = tranform_cp.translation.z
        x = tranform_cp.rotation.x
        y = tranform_cp.rotation.y
        z = tranform_cp.rotation.z
        w = tranform_cp.rotation.w
        T = np.zeros((4, 4))
        r = Rot.from_quat([x, y, z, w])
        T[3, 3] = 1
        T[0:3, 0:3] = r.as_matrix()
        T[0, 3] = sx
        T[1, 3] = sy
        T[2, 3] = sz
        self.T_ecm = T

    def sujpsm2_sub(self, msg):
        tranform_cp = msg.transform
        sx = tranform_cp.translation.x
        sy = tranform_cp.translation.y
        sz = tranform_cp.translation.z
        x = tranform_cp.rotation.x
        y = tranform_cp.rotation.y
        z = tranform_cp.rotation.z
        w = tranform_cp.rotation.w
        T = np.zeros((4, 4))
        r = Rot.from_quat([x, y, z, w])
        T[3, 3] = 1
        T[0:3, 0:3] = r.as_matrix()
        T[0, 3] = sx
        T[1, 3] = sy
        T[2, 3] = sz
        self.T_sujpsm2 = T

    def camera1_sub(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        try:
            self.camera1_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            # print('camera1 read')
        except CvBridgeError as e:
            print(e)

    def camera2_sub(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        try:
            self.camera2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            # print("camera2 read")
        except CvBridgeError as e:
            print(e)

    def robot_sub(self, msg):
        self.robot_status = msg.data

    def record_sub(self, msg):
        self.record_status = msg.data

    def pa_sub(self, msg):
        self.pa_status = msg.data

    def get_data(self):

        T_ecmoffset = -np.eye(4)
        T_ecmoffset[3, 3] = 1
        T_psmoffset = np.zeros((4, 4))
        T_psmoffset[3, 3] = 1
        # T_psmoffset[1, 3] = 0.49
        T_psmoffset[1, 3] = 0.0
        T_psmoffset[0, 0] = 1
        T_psmoffset[2, 1] = 1
        T_psmoffset[1, 2] = -1

        T_ecm_psm2 = self.T_sujpsm2 @ T_psmoffset @ self.T_psm2
        # T_suj_ecmtip = self.T_sujecm @ self.T_ecm
        # T_suj_psm2tip = self.T_sujpsm2 @ self.T_psm2
        # T_ecm_psm2 = np.linalg.inv(T_suj_ecmtip) @ T_suj_psm2tip
        return T_ecm_psm2, self.camera1_img, self.camera2_img

    def save_json(self, file_path, T):
        config = {'ecm_psm_transform': copy.deepcopy(T.tolist()),
                  'suj_ecm_transform': copy.deepcopy(self.T_sujecm.tolist()),
                  'suj_psm2_transform': copy.deepcopy(self.T_sujpsm2.tolist()),
                  'ecm_transform': copy.deepcopy(self.T_ecm.tolist()),
                  'psm2_transform': copy.deepcopy(self.T_psm2.tolist())}
        json_w = json.dumps(config)
        f = open(file_path, 'w')
        f.write(json_w)
        f.close()

    def save_img(self, folder_path, img_left, img_right):
        img_left = copy.deepcopy(img_left)
        img_right = copy.deepcopy(img_right)
        img_left_name = f'{self.count}_left.jpg'
        img_right_name = f'{self.count}_right.jpg'
        img_left_path = os.path.join(folder_path, img_left_name)
        img_right_path = os.path.join(folder_path, img_right_name)
        cv2.imwrite(img_left_path, img_left)
        cv2.imwrite(img_right_path, img_right)

    def sub_run(self, feq):
        rate = rospy.Rate(feq)
        while not rospy.is_shutdown():
            T_ecm_psm2, img_left, img_right = self.get_data()
            if self.pa_status and (not self.robot_status):
                print('saving')
                self.count += 1
                file_json_name = f'{self.count}.json'
                file_json_path = os.path.join(transform_folder, file_json_name)
                self.save_json(file_json_path, T_ecm_psm2)
                self.save_img(img_folder, img_left, img_right)
            rate.sleep()


if __name__ == '__main__':
    refresh_rate = 5
    # refresh_rate = 1
    sub_class = triadRecorder()
    sub_class.sub_run(refresh_rate)


