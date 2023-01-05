% =============================================================================
% file name:    renderPA2ECM.m
% description:  render PA image to ecm view based on tracked marker
% author:       Xihan Ma
% date:         2022-12-30
% =============================================================================
clc; clear; close all;

%% ----------------- ROS network -----------------
rosshutdown
ros_master_uri = 'http://192.168.1.2:11311';
setenv('ROS_MASTER_URI',ros_master_uri) % ip of robot desktop

% local_ip = '130.215.192.168';
% local_ip = getenv('ROS_IP')
% setenv('ROS_IP',local_ip)   % ip of this machine
rosinit

ecm_cam1_sub = rossubscriber('Clarius/US', 'sensor_msgs/Image');

rate = rateControl(20);
while true
  ecm_cam1_msg = receive(ecm_cam1_sub);
  ecm_cam1_img = reshape(ecm_cam1_msg.Data, ecm_cam1_msg.Height, ecm_cam1_msg.Width);
  disp(size(ecm_cam1_img))
  waitfor(rate);
end

