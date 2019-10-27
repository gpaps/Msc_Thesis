# if ids is not None:
#     for i in range(len(ids)):
#         c = corners[i][0]
#         plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "+", label="id={0}".format(ids[i]))
# """for points in rejectedImgPoints:
#     y = points[:, 0]
#     x = points[:, 1]
#     plt.plot(x, y, ".m-", linewidth = 1.)"""
# plt.legend()
# plt.show()

#DUMP
# if ret==True:
    #     rvecs, tvecs,rejectedImgPoints = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
    #     print(rvecs, "end_rotation")
    #     print(tvecs, "end_translation")
    #     # aruco.drawAxis(gray, camera_matrix, dist_coeffs, rvecs, tvecs, length=0.1)
    #         imaxis = aruco.drawAxis(gray, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.01)
    #     for i in range(len(tvecs)):
from goprocam import GoProCamera, constants
goproCamera = GoProCamera.GoPro()
goproCamera.shoot_video(10)