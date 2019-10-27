import numpy as np
import cv2
import cv2.aruco as aruco
import yaml

with open('true.yaml') as f:
    loadeddict = yaml.load(f)
camera_matrix = np.asarray(loadeddict.get('camera_matrix'))
dist_coeffs = np.asarray(loadeddict.get('distortion_coefficients'))

cap = cv2.VideoCapture(0)
markerLength = 50

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
    parameters = aruco.DetectorParameters_create()

    # print(parameters)

    # lists of ids and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(corners)
    # rvecs, tvecs, objpointsbg = aruco.estimatePoseSingleMarkers(corners,0.05,)
    if ids != None: # if aruco marker detected
        rvec, tvec, = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        gray = aruco.drawAxis(gray, camera_matrix, dist_coeffs, rvec, tvec, 100)



        gray = aruco.drawDetectedMarkers(gray, corners, ids)

        #print(rejectedImgPoints)
        # Display the resulting frame
        cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()