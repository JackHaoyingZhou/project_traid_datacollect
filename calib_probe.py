#! /usr/bin/env python3
# =============================================================================
# file name:    calib_probe.py
# description:  load saved ECM images, manually label aruco marker positions in ECM images
# author:       Xihan Ma
# date:         2022-09-24
# =============================================================================
import os
import cv2
import csv
import time
import numpy as np
from cv2 import aruco
from scipy.io import savemat
from datetime import datetime


class StreamECMData():
  IMG_RAW_HEIGHT = 1080   # original size
  IMG_RAW_WIDTH = 1920
  # IMG_DISP_HEIGHT = 480
  # IMG_DISP_WIDTH = 640
  IMG_DISP_HEIGHT = 1080
  IMG_DISP_WIDTH = 1920

  def __init__(self, img_path) -> None:
    self.img_path = img_path
    self.cam1_imgs = None
    self.cam2_imgs = None
    files = os.listdir(self.img_path)
    files = sorted(files, key=lambda x: int(x.split("_")[0]))  # sort on first part of str
    self.cam1_files = []
    self.cam2_files = []
    for f in files:
      if 'right' in f:
        self.cam1_files.append(f)
      elif 'left' in f:
        self.cam2_files.append(f)
    assert (len(self.cam1_files) == len(self.cam2_files) and len(self.cam1_files) > 0)
    self.num_imgs = len(self.cam1_files)
    t_start = time.perf_counter()
    self.load_cam1_imgs()
    self.load_cam2_imgs()
    t_stop = time.perf_counter()
    print(f'finish loading ecm images, total time elapsed: {t_stop-t_start:.2f} sec.')

  def frm_preproc(self, img: np.ndarray) -> np.ndarray:
    # kernel = np.array([[-1, -1, -1],
    #                    [-1, 9, -1],
    #                    [-1, -1, -1]])
    # img = cv2.filter2D(img, -1, kernel)
    # img = cv2.GaussianBlur(img, (5, 5), 0.8)
    # kernel = np.ones((3, 3), dtype=np.uint8)
    # img = cv2.dilate(img, kernel, iterations=1)
    return img

  def load_cam1_imgs(self) -> None:
    self.cam1_imgs = np.zeros((self.num_imgs, self.IMG_DISP_HEIGHT, self.IMG_DISP_WIDTH, 3), dtype=np.uint8)
    for i, f in enumerate(self.cam1_files):
      # print(f'read cam1: {f} ({i+1}/{self.num_imgs})')
      cam1_img_raw = cv2.imread(os.path.join(self.img_path, f))
      cam1_img_raw = self.frm_preproc(cam1_img_raw)
      self.cam1_imgs[i, :, :, :] = cv2.resize(cam1_img_raw, (self.IMG_DISP_WIDTH, self.IMG_DISP_HEIGHT),
                                              interpolation=cv2.INTER_AREA)

  def load_cam2_imgs(self) -> None:
    self.cam2_imgs = np.zeros((self.num_imgs, self.IMG_DISP_HEIGHT, self.IMG_DISP_WIDTH, 3), dtype=np.uint8)
    for i, f in enumerate(self.cam2_files):
      # print(f'read cam2: {f} ({i+1}/{self.num_imgs})')
      cam2_img_raw = cv2.imread(os.path.join(self.img_path, f))
      cam2_img_raw = self.frm_preproc(cam2_img_raw)
      self.cam2_imgs[i, :, :, :] = cv2.resize(cam2_img_raw, (self.IMG_DISP_WIDTH, self.IMG_DISP_HEIGHT),
                                              interpolation=cv2.INTER_AREA)

  def get_cam1_img(self, nfrm: int) -> np.ndarray:
    assert (nfrm <= self.num_imgs)
    return self.cam1_imgs[nfrm, :, :, :]

  def get_cam2_img(self, nfrm: int) -> np.ndarray:
    assert (nfrm <= self.num_imgs)
    return self.cam2_imgs[nfrm, :, :, :]


class TrackMarker:
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

  def __del__(self):
    if self.doRec:
      self.pix_file_out.close()
      self.pos_file_out.close()

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
    if self.doRec:
      self.save_mk_pix()

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
        rmat = -1*np.ones([3, 3])  # if not detected
        tvec = -1*np.ones([1, 3])
      self.mk_rmat[id, :] = rmat.flatten()
      self.mk_tvec[id, :] = tvec
    if self.doRec:
      self.save_mk_pos()

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


def auto_detect():
  ecm = StreamECMData(img_folder)
  mk_obj = TrackMarker(num_mk=2, recording=False)

  cv2.namedWindow('ecm2', cv2.WINDOW_AUTOSIZE)
  for i in range(ecm.num_imgs):
    frame = ecm.get_cam2_img(i)

    mk_obj.detect_markers(frame)
    mk_obj.estimate_pose(frame, camera_matrix, dist_coeff)

    # dist_pix = mk_obj.calc_marker_dist_pix()
    # dist_xyz = mk_obj.calc_marker_dist_xyz()
    # if dist_pix != -1 and dist_xyz != -1:
    #   print(f'dist in img: {dist_pix:.3f} [pix], dist in xyz: {dist_xyz*1000:.4f} [mm]')

    print(f'rot: \n {mk_obj.mk_rmat.reshape((2,3,3))} \ntrans: \n {mk_obj.mk_tvec}')

    cv2.imshow('ecm2', mk_obj.mk_bbox_frame)
    # cv2.imshow('ecm2', mk_obj.mk_axis_frame)

    key = cv2.waitKey(10)
    if key & 0xFF == ord('q') or key == 27:
      break
    time.sleep(0.5)


def on_mouse_click(event, x, y, flags, param):
  global mk_counter, mk_pix
  if event == cv2.EVENT_LBUTTONDBLCLK:
    cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)
    assert (mk_counter >= 0 and mk_counter < 4)
    mk_pix[mk_counter, :] = [x, y]
    mk_counter += 1
    if mk_counter > 4-1:  # reset marker counter after labeling 3 markers
      mk_counter = 0
    cv2.imshow('ecm2', frame)


camera_matrix = np.array([[1811.1, 0.0, 813.3], [0.0, 1815.3, 781.8], [0.0, 0.0, 1.0]])  # ECM cam2
dist_coeff = np.array([-0.3494, 0.4607, 0.0, 0.0])  # ECM cam2
# ========== choose dataset ==========
# data_folder = '9-23-2022/T2_pa700/'
# data_folder = '9-23-2022/T2_pa820/'
# data_folder = '9-23-2022/T3_pa700/'
# data_folder = '9-23-2022/T3_pa820/'
# data_folder = '10-20-2022/T10_pa850/'
data_folder = '10-20-2022/T10_us/'
# ====================================
img_folder = os.path.join(data_folder, 'image/')
assert (os.path.exists(img_folder) and os.path.exists(data_folder))
assert (os.path.exists(data_folder+'labeled/'))  # directory to store labeled data, need to be manually created
mk_counter = 0
mk_pix = -1*np.ones((4, 2), dtype=np.int32)  # 4 markers by default


if __name__ == "__main__":
  doManualLabel = False

  if doManualLabel is False:
    auto_detect()
  else:
    # ========== manual detect ==========
    ecm = StreamECMData(img_folder)
    mk_pix_rec = -1*np.ones((ecm.num_imgs, 4, 2), dtype=np.int32)
    cv2.namedWindow('ecm2', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('ecm2', on_mouse_click)

    i = 0
    try:
      while 1:
        frame = ecm.get_cam2_img(i)
        cv2.imshow('ecm2', frame)
        key = cv2.waitKey(0)
        if key & 0xFF == ord('q') or key == 27:
          break
        elif key == ord('s'):
          mk_pix_rec[i, :, :] = mk_pix
          print(f'({i+1}/{ecm.num_imgs}) pix positions: mk0 {mk_pix[0]}, mk1 {mk_pix[1]}, mk2 {mk_pix[2]}, mk3 {mk_pix[3]}')
          pic_path = os.path.join(data_folder, 'labeled/', '{}_labeled.png'.format(str(i+1)))
          cv2.imwrite(pic_path, frame)
          i += 1
          mk_counter = 0

        if i >= ecm.num_imgs:
          print('finished labeling')
          mdic = {"mk0": mk_pix_rec[:, 0, :].reshape(ecm.num_imgs, 2),
                  "mk1": mk_pix_rec[:, 1, :].reshape(ecm.num_imgs, 2),
                  "mk2": mk_pix_rec[:, 2, :].reshape(ecm.num_imgs, 2),
                  "mk3": mk_pix_rec[:, 3, :].reshape(ecm.num_imgs, 2)}
          mat_path = os.path.join(data_folder, 'labeled/mk_pix.mat')
          savemat(mat_path, mdic)
          break
    except Exception as e:
      print(e)
    finally:
      mdic = {"mk0": mk_pix_rec[:, 0, :].reshape(ecm.num_imgs, 2),
              "mk1": mk_pix_rec[:, 1, :].reshape(ecm.num_imgs, 2),
              "mk2": mk_pix_rec[:, 2, :].reshape(ecm.num_imgs, 2),
              "mk3": mk_pix_rec[:, 3, :].reshape(ecm.num_imgs, 2)}
      mat_path = os.path.join(data_folder, 'labeled/mk_pix.mat')
      savemat(mat_path, mdic)
    # ===================================
