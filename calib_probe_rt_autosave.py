#! /usr/bin/env python3
# =============================================================================
# file name:    calib_probe_rt_autosave.py
# description:  stream ECM images from ROS topic, estimate ECM-to-probe pose using aruco markers,
# automatically save on flag published in topic
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
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

np.set_printoptions(precision=3)
camera_matrix = np.array([[1811.1, 0.0, 813.3], [0.0, 1815.3, 781.8], [0.0, 0.0, 1.0]])  # ECM cam2
dist_coeff = np.array([-0.3494, 0.4607, 0.0, 0.0])  # ECM cam2


class TrackMarker:
  __detect_param = aruco.DetectorParameters_create()
  pix_save_path = os.path.join(os.path.dirname(__file__), 'data/{}_marker_pix_log.csv'.format(datetime.now()))
  pos_save_path = os.path.join(os.path.dirname(__file__), 'data/{}_marker_pos_log.csv'.format(datetime.now()))

  def __init__(self, num_mk=2, recording=False):
    ''' detect markers
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

  def __del__(self):
    if self.doRec:
      self.pix_file_out.close()
      self.pos_file_out.close()

  def get_marker_pix(self):
    return self.mk_pix

  def detect_markers(self, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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
    # if self.doRec:
    #   self.save_mk_pix()

  def estimate_pose(self, frame, cam_intri, dist_coeff):
    self.mk_axis_frame = frame.copy()
    for id in range(self.num_markers):
      try:
        loc_marker = self.corners[np.where(self.ids == id)[0][0]]
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(loc_marker, 0.047, cam_intri, dist_coeff)
        self.mk_axis_frame = aruco.drawAxis(self.mk_axis_frame, camera_matrix, dist_coeff, rvecs, tvecs, 0.05)
        rmat = cv2.Rodrigues(rvecs)[0]       # 3x3 rotation
        tvec = np.transpose(tvecs)[:, 0, 0]  # 3x1 translation
      except Exception as e:
        # print(e)
        self.mk_axis_frame = frame.copy()
        rmat = -1*np.ones([3, 3])  # if not detected
        tvec = -1*np.ones([1, 3])
      self.mk_rmat[id, :] = rmat.flatten()
      self.mk_tvec[id, :] = tvec
    # if self.doRec:
    #   self.save_mk_pos()

  def calc_marker_dist_pix(self):
    if self.num_markers == 2:
      if (self.mk_pix != -1).all():
        dist = np.sqrt((self.mk_pix[0, 0]-self.mk_pix[1, 0])**2 +
                       (self.mk_pix[0, 1]-self.mk_pix[1, 1])**2)
      else:
        dist = -1
      return dist
    else:
      return -1

  def calc_marker_dist_xyz(self):
    if self.num_markers == 2:
      if (self.mk_tvec != -1).all():
        dist = np.sqrt((self.mk_tvec[0, 0]-self.mk_tvec[1, 0])**2 +
                       (self.mk_tvec[0, 1]-self.mk_tvec[1, 1])**2 +
                       (self.mk_tvec[0, 2]-self.mk_tvec[1, 2])**2)
      else:
        dist = -1
      return dist
    else:
      return -1

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


class StreamECMData():
  IMG_RAW_HEIGHT = 1080   # original size
  IMG_RAW_WIDTH = 1920
  IMG_DISP_HEIGHT = 480
  IMG_DISP_WIDTH = 640

  def __init__(self) -> None:
    rospy.init_node('ECM_stream_subscriber', anonymous=True)
    self.cam1_img = np.zeros((self.IMG_DISP_HEIGHT, self.IMG_DISP_WIDTH, 3), dtype=np.uint8)
    self.cam2_img = np.zeros((self.IMG_DISP_HEIGHT, self.IMG_DISP_WIDTH, 3), dtype=np.uint8)
    self.record_flag = False  # flag to start recording marker pose
    self.ecm_cam1_sub = rospy.Subscriber('/cv_camera1/image_raw', Image, self.ecm_cam1_cb)
    self.ecm_cam2_sub = rospy.Subscriber('/cv_camera2/image_raw', Image, self.ecm_cam2_cb)
    self.record_flag_sub = rospy.Subscriber('/record_ready_flag', Bool, self.record_flag_cb, queue_size=5000)

    self.mk_obj = TrackMarker(num_mk=4, recording=True)
    self.frm_save_count = 0
    self.isMkRecorded = False

  def frm_preproc(self, img):
    """ preprocessing for better marker visualization
    """
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

  def record_flag_cb(self, msg: Bool) -> None:
    self.record_flag = msg.data
    if self.record_flag:
      print(f'flag callback: {self.record_flag}')
      if not self.isMkRecorded:
        for i in range(50):
          mk_pix = self.mk_obj.get_marker_pix()
          num_detected = np.sum(np.all(mk_pix != -1, axis=1))
          print(f'markers detected: {num_detected}')
          if num_detected > 0:
            self.mk_obj.save_mk_pix()
            print(f'marker recorded:\n{mk_pix}')
            frm_save_path = 'data/mk_frame{}.png'.format(self.frm_save_count)
            cv2.imwrite(os.path.join(os.path.dirname(__file__), frm_save_path), self.mk_obj.mk_bbox_frame)
            print(f'marker frame {self.frm_save_count} saved')
            self.frm_save_count += 1
            self.isMkRecorded = True
            break
          print('too less marker detected')
    else:
      # print(f'record not ready')
      self.isMkRecorded = False

  def get_cam1_img(self) -> np.ndarray:
    return self.cam1_img

  def get_cam2_img(self) -> np.ndarray:
    return self.cam2_img

  def get_record_flag(self) -> bool:
    return self.record_flag

  def detect_mk_from_ecm(self) -> None:
    cv2.namedWindow('ecm2', cv2.WINDOW_AUTOSIZE)
    isMkRecorded = False
    frm_save_count = 0
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
      self.mk_obj.detect_markers(self.cam2_img)
      cv2.imshow('ecm2', self.mk_obj.mk_bbox_frame)
      key = cv2.waitKey(1)
      if key & 0xFF == ord('q') or key == 27:
        break
      rate.sleep()


def main():
  ecm = StreamECMData()

  ecm.detect_mk_from_ecm()

  # mk_obj = TrackMarker(num_mk=4, recording=True)
  # # cv2.namedWindow('ecm2', cv2.WINDOW_AUTOSIZE)
  # rate = rospy.Rate(1000)

  # isMkRecorded = False
  # frm_save_count = 0
  # while not rospy.is_shutdown():
  #   frame = ecm.get_cam2_img()
  #   mk_obj.detect_markers(frame)
  #   # mk_obj.estimate_pose(frame, camera_matrix, dist_coeff)

  #   # dist_pix = mk_obj.calc_marker_dist_pix()
  #   # dist_xyz = mk_obj.calc_marker_dist_xyz()
  #   # if dist_pix != -1 and dist_xyz != -1:
  #   #   print(f'dist in img: {dist_pix:.3f} [pix], dist in xyz: {dist_xyz*1000:.4f} [mm]')

  #   # print(f'rot: \n {mk_obj.mk_rmat.reshape((2,3,3))} \ntrans: \n {mk_obj.mk_tvec}')

  #   # ===== comment imshow to save IO =====
  #   cv2.imshow('ecm2', mk_obj.mk_bbox_frame)
  #   # cv2.imshow('ecm2', mk_obj.mk_axis_frame)
  #   # =====================================

  #   record_ready_flag = ecm.record_flag
  #   # print(f'record ready flag: {record_ready_flag}')
  #   if record_ready_flag:
  #     if not isMkRecorded:
  #       for i in range(50):
  #         mk_pix = mk_obj.get_marker_pix()
  #         num_detected = np.sum(np.all(mk_pix != -1, axis=1))
  #         print(f'markers detected: {num_detected}')
  #         if num_detected > 0:
  #           mk_obj.save_mk_pix()
  #           print(f'marker recorded:\n{mk_pix}')
  #           frm_save_path = 'data/mk_frame{}.png'.format(frm_save_count)
  #           cv2.imwrite(os.path.join(os.path.dirname(__file__), frm_save_path), mk_obj.mk_bbox_frame)
  #           print(f'marker frame {frm_save_count} saved')
  #           frm_save_count += 1
  #           isMkRecorded = True
  #           break
  #         print('too less marker detected')
  #   else:
  #     # print(f'record not ready')
  #     isMkRecorded = False

  #   key = cv2.waitKey(1)
  #   if key & 0xFF == ord('q') or key == 27:
  #     break
  #   elif key == ord('s'):
  #     frm_save_path = 'data/mk_frame{}.png'.format(frm_save_count)
  #     frm_save_count += 1
  #     cv2.imwrite(os.path.join(os.path.dirname(__file__), frm_save_path), mk_obj.mk_bbox_frame)
  #     mk_obj.save_mk_pix()
  #     print(f'marker frame {frm_save_count} saved:\n{mk_obj.get_marker_pix()}')
  #   rate.sleep()


if __name__ == "__main__":
  main()
