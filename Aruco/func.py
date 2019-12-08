import os
import cv2
import yaml
import cv2.aruco as aruco
from matplotlib import pyplot as plt
from scipy import linalg
import pydot as dot

def aruco_gene(aruco_dict, iterations, sidepixels):
    '''
       aruco_gene(...)
            aruco_gene(dictionary, Iterations(ids),
             sidePixels[, img[, borderBits]]) -> img
    '''
    for i in range(iterations):
        img1 = aruco.drawMarker(aruco_dict, i, sidepixels)
        cv2.imwrite("{}.jpg".format(i), img1)


class Camera(object):
    """ Class for representing pin-hole cameras. """
    def __init__(self, P):
        """ Initialize P = K[R|t] camera model. """
        self.P = P
        self.K = None  # calibration matrix
        self.R = None  # rotation
        self.t = None  # translation
        self.c = None  # camera center

    def project(self,X):
        """ Project points in X (4*n array) and normalize coordinates. """
        x = dot(self.P, X)
        for i in range(3):
            x[i] /= x[2]
        return x
