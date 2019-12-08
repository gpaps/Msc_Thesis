import os
import numpy as np
import cv2
import cv2.aruco as aruco
# from Msc_Thesis.Aruco.func import aruco_gene
from matplotlib import pyplot as plt
import yaml

# importing aruco dictionary
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)

# Aruco_generator_ | Create_Markers | its_def |
# aruco_gene(aruco_dict, 11, 200)
#/home/evangeloit/Desktop/py-msc/Calibration/Videos_Imgs/GoProHero7/known
# Video-Capture
cap = cv2.VideoCapture('/home/evangeloit/Desktop/py-msc/Calibration/Videos_Imgs/GoProHero7/known/GH010298.MP4')
# cap = cv2.VideoCapture(0)

# Calibration parameters
calibrationFile = "gopro7b.yaml"
calibrationParams = cv2.FileStorage(calibrationFile, cv2.FILE_STORAGE_READ)
camera_matrix = calibrationParams.getNode("camera_matrix").mat()
dist_coeffs = calibrationParams.getNode("distortion_coefficients").mat()

if camera_matrix is None or dist_coeffs is None:
        print("Calibration issue. Remove ./file.yaml and recalibrate your camera with CalibrateCamera.py.")
        exit()

# side length of the marker in meter
markerLength = 0.53  # Measurement unit is metre.(cm =5.3)
arucoParams = cv2.aruco.DetectorParameters_create()

# Create Null(None)-vectors using for rotations and translations for postures
tvecs, rvecs = None, None

while True:

    # Capture frame-by-frame
    ret, frame = cap.read()#;print(frame.shape)  #480x640

    # Operations on the frame comes here
    if ret is True:
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = frame
    else:
        continue

    # Lists of id's and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams)
    print(corners)  # print(arucoParams) # print(rejectedImgPoints)

    gray = aruco.drawDetectedMarkers(gray, corners, ids, borderColor=None)  # paint

    if ids is not None and len(ids) > 0:

        # Estimate the posture per each Aruco marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        np.save = ('rvecs1', rvecs)
        np.save = ('tvecs1', tvecs)
        for rvec, tvec in zip(rvecs, tvecs):
            gray = aruco.drawAxis(gray, camera_matrix, dist_coeffs, rvec, tvec, .2)

    # Display the resulting frame
    cv2.imshow('frame', gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
