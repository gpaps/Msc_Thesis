import os
import cv2
import yaml
import cv2.aruco as aruco
from matplotlib import pyplot as plt

def aruco_gene(aruco_dict, iterations, sidepixels):
    '''
       aruco_gene(...)
            aruco_gene(dictionary, Iterations(ids),
             sidePixels[, img[, borderBits]]) -> img
    '''
    for i in range(iterations):
        img1 = aruco.drawMarker(aruco_dict, i, sidepixels)
        cv2.imwrite("{}.jpg".format(i), img1)
