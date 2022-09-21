#! /usr/bin/env python3
'''
file name:    calib_probe.py
description:  ECM-probe pose estimation using aruco markers
author:       Xihan Ma
date:         2022-09-20
'''
import os
import csv
import time
import rospy
import numpy as np
from cv2 import cv2
from cv2 import aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class StreamECMData():
  IMG_RAW_HEIGHT = 1080   # original size
  IMG_RAW_WIDTH = 1920
  IMG_DISP_HEIGHT = 640
  IMG_DISP_WIDTH = 480

  def __init__(self) -> None:
    rospy.init_node('ECM_stream_subscriber', anonymous=True)
    self.ecm_cam1_sub = rospy.Subscriber('cv_camera1/image_raw', Image, self.ecm_cam1_cb)
    self.ecm_cam2_sub = rospy.Subscriber('cv_camera2/image_raw', Image, self.ecm_cam2_cb)
    self.cam1_img = np.zeros((self.IMG_DISP_HEIGHT, self.IMG_DISP_WIDTH, 3), dtype=np.uint8)
    self.cam2_img = np.zeros((self.IMG_DISP_HEIGHT, self.IMG_DISP_WIDTH, 3), dtype=np.uint8)

  def ecm_cam1_cb(self, msg: Image) -> None:
    cam1_img_raw = CvBridge().imgmsg_to_cv2(msg).astype(np.uint8)
    self.cam1_img = cv2.resize(cam1_img_raw, (self.IMG_DISP_HEIGHT, self.IMG_DISP_WIDTH),
                               interpolation=cv2.INTER_AREA)

  def ecm_cam2_cb(self, msg: Image) -> None:
    cam2_img_raw = CvBridge().imgmsg_to_cv2(msg).astype(np.uint8)
    self.cam2_img = cv2.resize(cam2_img_raw, (self.IMG_DISP_HEIGHT, self.IMG_DISP_WIDTH),
                               interpolation=cv2.INTER_AREA)

  def get_cam1_img(self) -> np.ndarray:
    return self.cam1_img

  def get_cam2_img(self) -> np.ndarray:
    return self.cam2_img


class TrackMarker:
  __detect_param = aruco.DetectorParameters_create()
  # save_path = os.path.join(os.path.dirname(__file__), 'data/marker_pos_log.csv')

  def __init__(self, num_mk=1, dict_id=4):
    if dict_id == 6:
      self.__aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    elif dict_id == 4:
      self.__aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    self.num_markers = num_mk
    self.mk_bbox_frame = None
    self.mk_axis_frame = None
    self.corners = None
    self.ids = None
    self.mk_pos = np.zeros((self.num_markers, 2), dtype=int)
    self.mk_rmat = None
    self.mk_tvec = None
    # self.file_out = open(self.save_path, 'a')
    # self.writer = csv.writer(self.file_out)

  def detect_markers(self, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    self.corners, self.ids, _ = aruco.detectMarkers(
        gray, self.__aruco_dict, parameters=self.__detect_param)
    # print(self.ids)
    self.mk_bbox_frame = aruco.drawDetectedMarkers(frame.copy(), self.corners, self.ids)
    for id in range(self.num_markers):
      try:
        curr_mk = self.corners[np.where(self.ids == id)[0][0]][0]
        self.mk_pos[id, :] = [round(curr_mk[:, 0].mean()), round(curr_mk[:, 1].mean())]
      except:
        self.mk_pos[id, :] = [-1, -1]  # if marker is not detected
      # print('id:', id+1, ' pos', self.mk_pos[id, :])

  def estimate_pose(self, frame, cam_intri, dist_coeff, mk_id=0):
    try:
      # print(self.corners[0])
      # loc_marker = self.corners[np.where(self.ids == mk_id)[0][0]]
      loc_marker = self.corners[np.where(self.ids == mk_id)[0][0]]
      rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(loc_marker, 0.047, cam_intri, dist_coeff)
      self.mk_axis_frame = aruco.drawAxis(frame.copy(), camera_matrix, dist_coeff, rvecs, tvecs, 0.05)
      self.rmat = cv2.Rodrigues(rvecs)[0]       # 3x3 rotation
      self.tvec = np.transpose(tvecs)[:, 0, 0]  # 3x1 translation
      UV = [loc_marker[0][:, 0].mean(), loc_marker[0][:, 1].mean()]
    except Exception as e:
      print(e)
      self.mk_axis_frame = frame
      self.rmat = -1*np.ones([3, 3])
      self.tvec = [-1, -1, -1]
      UV = [-1, -1]
    # print('rot: \n', self.rmat)
    # print('trans: \n', self.tvec)
    # return UV, rmat[:, 2], tvec
    return UV

  def save_mk_pos(self):
    row2write = list()
    for id in range(self.num_markers):
      row2write.append((self.mk_pos[id, 0], self.mk_pos[id, 1]))
    self.writer.writerow(row2write)

  def save_mk_pos_xyz(self, pnts):
    row2write = list()
    # append markers in pixels
    for id in range(self.num_markers):
      row2write.append((self.mk_pos[id, 0], self.mk_pos[id, 1]))
    # append targets in xyz
    for id in range(self.num_markers):
      row2write.append((pnts[id, 0], pnts[id, 1], pnts[id, 2]))
    self.writer.writerow(row2write)


# camera_matrix = np.array([[662.1790, 0.0, 322.3619], [0.0, 662.8344, 252.0131], [0.0, 0.0, 1.0]]) # realsense
# dist_coeff = np.array([0.0430651, -0.1456001, 0.0, 0.0])  # realsense
camera_matrix = np.array([[1811.1, 0.0, 813.3], [0.0, 1815.3, 781.8], [0.0, 0.0, 1.0]])  # ECM cam2
dist_coeff = np.array([-0.3494, 0.4607, 0.0, 0.0])  # ECM cam2


def main():
  frm_save_path = '../data/densepose/mk_frame.png'

  ecm = StreamECMData()
  mk_obj = TrackMarker(num_mk=2, dict_id=4)
  rate = rospy.Rate(20)
  cv2.namedWindow('ecm2', cv2.WINDOW_AUTOSIZE)

  while not rospy.is_shutdown():
    frame = ecm.get_cam2_img()

    mk_obj.detect_markers(frame)
    mk_obj.estimate_pose(frame, camera_matrix, dist_coeff)

    tar_pix = np.zeros((mk_obj.num_markers, 2), dtype=int)
    for i in range(mk_obj.num_markers):
      tar_pix[i, :] = [mk_obj.mk_pos[i, 0], mk_obj.mk_pos[i, 1]]

    cv2.imshow('ecm2', mk_obj.mk_axis_frame)
    # cv2.imshow('ecm2', mk_obj.mk_bbox_frame)
    print(mk_obj.mk_pos)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
      break
    elif key == ord('s'):
      cv2.imwrite(os.path.join(os.path.dirname(__file__), frm_save_path), mk_obj.mk_bbox_frame)
      print('marker frame saved')
    rate.sleep()


if __name__ == "__main__":
  main()
