#! /usr/bin/env python3
# =============================================================================
# file name:    calib_probe_rt.py
# description:  stream ECM images from ROS topic, estimate ECM-to-probe pose using aruco markers
# author:       Xihan Ma
# date:         2022-09-20
# =============================================================================
import os
import cv2
import csv
import rospy
import numpy as np
from cv2 import aruco
from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Int32MultiArray, Float32MultiArray

# camera_matrix = np.array([[1811.1, 0.0, 813.3], [0.0, 1815.3, 781.8], [0.0, 0.0, 1.0]])  # ECM cam2
camera_matrix = np.array([[596.5596, 0.0, 261.1265], [0.0, 790.1609, 238.1423], [0.0, 0.0, 1.0]])  # ECM cam2
dist_coeff = np.array([-0.3494, 0.4607, 0.0, 0.0])  # ECM cam2


class StreamECMData():
  '''
  '''

  def __init__(self) -> None:
    self.IMG_RAW_HEIGHT = 1080   # original size
    self.IMG_RAW_WIDTH = 1920
    self.IMG_DISP_HEIGHT = 480
    self.IMG_DISP_WIDTH = 640

    rospy.init_node('ECM_stream_subscriber', anonymous=True)
    # self.ecm_cam1_sub = rospy.Subscriber('/cv_camera1/image_raw', Image, self.ecm_cam1_cb)
    # self.ecm_cam2_sub = rospy.Subscriber('/cv_camera2/image_raw', Image, self.ecm_cam2_cb)
    self.ecm_cam1_sub = rospy.Subscriber('/cv_camera_right_2/image_raw', Image, self.ecm_cam1_cb)
    self.ecm_cam2_sub = rospy.Subscriber('/cv_camera_left_0/image_raw', Image, self.ecm_cam2_cb)

    self.cam1_img = np.zeros((self.IMG_DISP_HEIGHT, self.IMG_DISP_WIDTH, 3), dtype=np.uint8)
    self.cam2_img = np.zeros((self.IMG_DISP_HEIGHT, self.IMG_DISP_WIDTH, 3), dtype=np.uint8)

  def frm_preproc(self, img):
    # kernel = np.array([[-1, -1, -1],
    #                    [-1, 9, -1],
    #                    [-1, -1, -1]])
    # img = cv2.filter2D(img, -1, kernel)
    # img = cv2.GaussianBlur(img, (5, 5), 0.8)
    return img

  def ecm_cam1_cb(self, msg: Image) -> None:
    cam1_img_raw = CvBridge().imgmsg_to_cv2(msg).astype(np.uint8)
    cam1_img_raw = cv2.resize(cam1_img_raw, (self.IMG_DISP_WIDTH, self.IMG_DISP_HEIGHT),
                              interpolation=cv2.INTER_AREA)
    self.cam1_img = self.frm_preproc(cam1_img_raw)

  def ecm_cam2_cb(self, msg: Image) -> None:
    cam2_img_raw = CvBridge().imgmsg_to_cv2(msg).astype(np.uint8)
    cam2_img_raw = cv2.resize(cam2_img_raw, (self.IMG_DISP_WIDTH, self.IMG_DISP_HEIGHT),
                              interpolation=cv2.INTER_AREA)
    self.cam2_img = self.frm_preproc(cam2_img_raw)

  def get_cam1_img(self) -> np.ndarray:
    return self.cam1_img

  def get_cam2_img(self) -> np.ndarray:
    return self.cam2_img


class TrackMarker():
  __detect_param = aruco.DetectorParameters_create()
  pix_save_path = os.path.join(os.path.dirname(__file__), 'data/{}_marker_pix_log.csv'.format(datetime.now()))
  pos_save_path = os.path.join(os.path.dirname(__file__), 'data/{}_marker_pos_log.csv'.format(datetime.now()))

  def __init__(self, num_mk=1, recording=False):
    '''
    tested for less than two markers with ids being 0 and 1
    '''
    self.__aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    self.num_markers = num_mk
    self.mk_bbox_frame = None
    self.mk_axis_frame = None
    self.corners = None
    self.ids = None
    self.mk_pix = -1*np.ones((self.num_markers, 2), dtype=np.int32)
    self.mk_rmat = -1*np.ones((self.num_markers, 9), dtype=np.float32)
    self.mk_tvec = -1*np.ones((self.num_markers, 3), dtype=np.float32)
    # ===== save to file =====
    self.doRec = recording
    if self.doRec:
      self.pix_file_out = open(self.pix_save_path, 'w')
      self.pos_file_out = open(self.pos_save_path, 'w')
      self.pix_writer = csv.writer(self.pix_file_out)
      self.pos_writer = csv.writer(self.pos_file_out)
    # ===== pub to topic =====
    self.mk_pix_pub = rospy.Publisher('/marker/pix', Int32MultiArray, queue_size=1)
    self.mk_rot_pub = rospy.Publisher('/marker/rot', Float32MultiArray, queue_size=1)
    self.mk_trans_pub = rospy.Publisher('/marker/trans', Float32MultiArray, queue_size=1)
    self.mk_pix_msg = Int32MultiArray()
    self.mk_rot_msg = Float32MultiArray()
    self.mk_trans_msg = Float32MultiArray()

  def __del__(self):
    if self.doRec:
      self.pix_file_out.close()
      self.pos_file_out.close()

  def detect_markers(self, frame):
    gray = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
    self.corners, self.ids, _ = aruco.detectMarkers(
        gray, self.__aruco_dict, parameters=self.__detect_param)
    self.mk_bbox_frame = aruco.drawDetectedMarkers(frame.copy(), self.corners, self.ids)
    for id in range(self.num_markers):
      try:
        curr_mk = self.corners[np.where(self.ids == id)[0][0]][0]
        self.mk_pix[id, :] = [round(curr_mk[:, 0].mean()), round(curr_mk[:, 1].mean())]
      except Exception as e:
        # print(e)
        self.mk_pix[id, :] = [-1, -1]  # if not detected
    if self.doRec:
      self.save_mk_pix()

  def estimate_pose(self, frame, cam_intri, dist_coeff):
    self.mk_axis_frame = frame.copy()
    for id in range(self.num_markers):
      try:
        loc_marker = self.corners[np.where(self.ids == id)[0][0]]
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(loc_marker, 0.006, cam_intri, dist_coeff)
        self.mk_axis_frame = cv2.drawFrameAxes(self.mk_axis_frame, camera_matrix, dist_coeff, rvecs, tvecs, 0.01)  # cv2 has been updated
        rmat = cv2.Rodrigues(rvecs)[0]       # 3x3 rotation
        tvec = np.transpose(tvecs)[:, 0, 0]  # 3x1 translation
      except Exception as e:
        # print(f'pose estimation err: {e}')
        self.mk_axis_frame = frame.copy()
        rmat = -1*np.ones([3, 3])  # if not detected
        tvec = -1*np.ones([1, 3])
      self.mk_rmat[id, :] = rmat.flatten()
      self.mk_tvec[id, :] = tvec
    if self.doRec:
      self.save_mk_pos()

  def pub_mk_pix(self):
    self.mk_pix_msg.data = self.mk_pix.flatten()
    self.mk_pix_msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label='mk', size=self.num_markers),
                                                   MultiArrayDimension(label='coord', size=2)])
    self.mk_pix_pub.publish(self.mk_pix_msg)

  def pub_mk_pos(self):
    self.mk_rot_msg.data = self.mk_rmat.flatten()
    self.mk_rot_msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label='mk', size=self.num_markers),
                                                   MultiArrayDimension(label='rmat', size=9)])
    self.mk_trans_msg.data = self.mk_tvec.flatten()
    self.mk_trans_msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label='mk', size=self.num_markers),
                                                     MultiArrayDimension(label='tvec', size=3)])
    self.mk_rot_pub.publish(self.mk_rot_msg)
    self.mk_trans_pub.publish(self.mk_trans_msg)

  def save_mk_pix(self):
    row2write = list()
    for id in range(self.num_markers):
      row2write.append((self.mk_pix[id, 0], self.mk_pix[id, 1]))
    self.pix_writer.writerow(row2write)

  def save_mk_pos(self):
    row2write = list()
    # append rotation matrix
    for id in range(self.num_markers):
      row2write.append((self.mk_rmat[id, :]))
    # # append translation vector
    # for id in range(self.num_markers):
    #   row2write.append((self.mk_tvec[id, :]))
    self.pos_writer.writerow(row2write)


def main():
  mk_frm_save_path = 'data/mk_frame.png'
  frm_save_path = 'data/frame.png'

  ecm = StreamECMData()
  mk_obj = TrackMarker(num_mk=4, recording=False)
  rate = rospy.Rate(30)

  while not rospy.is_shutdown():
    frame = ecm.get_cam2_img()

    mk_obj.detect_markers(frame)
    mk_obj.estimate_pose(frame, camera_matrix, dist_coeff)
    mk_obj.pub_mk_pix()
    mk_obj.pub_mk_pos()

    # cv2.imshow('ecm2', frame)
    # cv2.imshow('ecm2', mk_obj.mk_bbox_frame)
    cv2.imshow('ecm2', mk_obj.mk_axis_frame)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
      break
    elif key == ord('s'):
      cv2.imwrite(os.path.join(os.path.dirname(__file__), frm_save_path), frame)
      print('resized frame saved')
      # cv2.imwrite(os.path.join(os.path.dirname(__file__), mk_frm_save_path), mk_obj.mk_bbox_frame)
      # print('marker frame saved')

    rate.sleep()


if __name__ == "__main__":
  main()
