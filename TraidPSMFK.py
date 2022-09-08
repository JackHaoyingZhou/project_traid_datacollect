#!/usr/bin/env python
############################################
## /* PSM FK for Traid project */ ##########
## \author: Haoying Zhou
############################################

import numpy as np

class TraidPSMFK():
    def __init__(self, joint_pos):
        ####### note: joint_pos should be a 1x3 matrix for each pose, if not, change one commented code below
        self.q = joint_pos
        self.screw_axis = np.array([[0,0,1,0,0,0],[-1,0,0,0,0,0],[0,0,0,0,-1,0]])
        self.l1 = 0.12
        self.l2 = 0.24
        self.l3 = 0.055
        self.l4 = 0.05
        self.theta_sin = 11/np.sqrt(221)
        self.theta_cos = 10/np.sqrt(221)

    #### get the transformation matrix for the end effector tip based on given info
    def __get_M(self):
        return np.array([[0,1,0,0],[self.theta_sin,0,-self.theta_cos,self.l1-self.l2-self.l3],[-self.theta_cos,0,-self.theta_sin,self.l4],[0,0,0,1]])

    ### generate skew symmetric matrix from a 3x1 vector
    @staticmethod
    def __skew(vector):
        assert len(vector)==3, 'The vector should be a 3 by 1 vector'
        skew_mtx = np.zeros((3,3))
        skew_mtx[0,1] = -vector[2]
        skew_mtx[0,2] = vector[1]
        skew_mtx[1,0] = vector[2]
        skew_mtx[1,2] = -vector[0]
        skew_mtx[2,0] = -vector[1]
        skew_mtx[2,1] = vector[0]
        return skew_mtx

    ### transfer 3x1 axis angle vector to 3x3 rotation matrix
    def __axisanlge2rot(self,omega,theta):
        omega_skew = self.__skew(omega)
        R = np.eye(3) + np.sin(theta)*omega_skew + (1-np.cos(theta))*np.dot(omega_skew,omega_skew)
        return R


    ### calculate transformation matrix based on twist
    def __twist2ht(self,S,theta):
        omega = S[0:3]
        v = S[3:6]
        T = np.zeros((4,4))
        T[3,3] = 1
        if np.linalg.norm(omega) < 1e-8:
            R = np.eye(3)
            p = np.transpose(v)*theta
        else:
            omega_skew = self.__skew(omega)
            R = self.__axisanlge2rot(omega,theta)
            p = np.dot((np.eye(3)*theta + (1-np.cos(theta))*omega_skew + (theta-np.sin(theta))*np.dot(omega_skew,omega_skew)),v)
        T[0:3, 0:3] = R
        T[0:3, 3] = p
        return T


    ### compute FK
    def compute_FK(self):
        M = self.__get_M()
        T = np.eye(4)
        for i_axis in range(self.screw_axis.shape[0]):
            S = self.screw_axis[i_axis,:]
            theta = self.q[0,i_axis]  ############ change based on q's shape
            T_i = self.__twist2ht(S,theta)
            T= np.dot(T,T_i)
        T = np.dot(T,M)
        return T

if __name__ == "__main__":
    q = np.zeros((1,3))
    q[0,1] = np.pi
    PSMFK = TraidPSMFK(q)
    T = PSMFK.compute_FK()
    print(T)


