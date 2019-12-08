import pybullet as p
# from OpenGL.arrays import GLbyteArray
import cv2
import time
import pybullet_data
import os
import pickle
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
# import pygame, pygame.image
# from pygame.locals import *

# Link to imported items
# link = '/home/gpaps/PycharmProjects/Msc_Thesis/bullet3-master/data/'

link = '/home/evangeloit/Desktop/py-msc/bullet3-2.88/data'
item_link = '/home/evangeloit/Desktop/py-msc/Msc_Thesis/PyBullet/Objects'

# Set Environment "Constants"
# physicsClient = p.connect(p.COV_ENABLE_GUI)
p.connect(p.GUI)

p.setGravity(0, 0, -9.81)
# planeId = p.loadURDF(link+"/plane100.urdf")

# Configure settings of the built-in OpenGL visualizer, such as enabling or disabling wireframe...This is useful since some laptops or Desktop GUIs have performance issues with our OpenGL 3 visualizer.
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 2)  # 0=False, 1=True(without/UI, 2=True(with/UI))

# Render - Dimension
Render_width = 1280
Render_height = 720

# Items Position URDFs - # https://www.andre-gaschler.com/rotationconverter/
cubeStartPos = [0.0, -0.3, 0.015]
cubeStartOrientation = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
wdn_cubeStartPos1 = [0.0, 0.7, 0.4] 			# [0, 5.2, 2.13]
wdn_cubeStartOrientation1 = p.getQuaternionFromEuler([0.0, 0.5, -1.6])  # ([0, 0.5, -1.6])

# Items - Object - Boxe's - Incline
r_box = p.loadURDF(item_link+"/cube.urdf", cubeStartPos, cubeStartOrientation,)
w_box = p.loadURDF(item_link+"/cube1.urdf", wdn_cubeStartPos1, wdn_cubeStartOrientation1)#, globalScaling=1)

# Import .obj-file(Incline-RigidB), Collision+Visual --> Multibody
shift = [-.5, -0.11, 0.0]
meshScale = [1, 1, 1]


# the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=item_link+"/inclinedCM100.obj",
										  collisionFramePosition=shift, meshScale=meshScale)

visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=item_link+"/inclinedCM100.obj",
									rgbaColor=[1, 1, 1, 0.9], specularColor=[0.4, 0.4, 0],
									visualFramePosition=shift, meshScale=meshScale)

incline = p.createMultiBody(baseMass=0, basePosition=[0, 0, 0.1],
							baseOrientation=[1, 1, 1, 1], baseInertialFramePosition=[0, 0, 0],
							baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex=visualShapeId,
							useMaximalCoordinates=True)

# ADDING Dynamics to Objects through  GUI
#p.changeDynamics(incline ,-1, lateralFriction= 0.5)
# Create UI -edita
inclineUI = p.addUserDebugParameter("Incline friction", 0, 1, 0.5)
r_boxUI = p.addUserDebugParameter("R_box friction", 0, 1, 0.5)
w_boxUI = p.addUserDebugParameter("Wd_Box friction", 0, 1, 0.8)
# spinningFrictionId = p.addUserDebugParameter("spinning friction", 0, 1, 0.03)


# Start with wireframe ON=1 off=0,
p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)


# counter and path to save pics in each session
# img_path ='/home/gpaps/PycharmProjects/Images'
img_path ='/home/evangeloit/Desktop/py-msc/Imgs'
snap = 0
# raw_input('Press enter to continue...')

# pickle-rick load K-matrix
with open("/home/evangeloit/Desktop/py-msc/Calibration/calib_02/wide_dist_pickle.p", "rb") as f:
    data = pickle.load(f)

K = data["mtx"]

# __Synthetic Camera Rendering__ #
# Compute _PROJECTION_ Matrix FOV
fx = K[0, 0]
fy = K[1, 1]
fov = 2*np.arctan(0.5*Render_height/fy)*180/np.pi
aspect = 16./9.
nearVal = 0.1
farVal = 100
fov_project_matrix = p.computeProjectionMatrixFOV(fov=fov, aspect=aspect, nearVal=nearVal, farVal=farVal)

# Compute - View - Matrix # _COMPUTE VIEW MATRIX_
cameraEyePosition = [1, 0, 0]
cameraTargetPosition = [0, 1, 0]
cameraUpVector = [0, 0, 1]
view_matrix = p.computeViewMatrix(cameraEyePosition, cameraTargetPosition, cameraUpVector)

# Compute - View - Matrix # _COMPUTE PROJECTION MATRIX_
# p.computeProjectionMatrix

# Set Camera position -observer- __ResetDebugVisualizerCamera__ and __COMPUTE VIEW MATRIX FROM YAW PITCH ROLL__
# cam_dist = 0.925  # 6
# cam_yaw = -270.80  # 90
# cam_pitch = -88.5  # 40
# cameraTargetPosition = [0.0, 0.4625, 0.0]
# # p.resetDebugVisualizerCamera(cam_dist, cam_yaw, cam_pitch, cameraTargetPosition)  #x,2,z
# base_pos, orn = p.getBasePositionAndOrientation(w_box)
# view_matrixRPY = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=base_pos, distance=cam_dist,
#                                             yaw=cam_yaw,
# 											pitch=cam_pitch,
# 											roll=0,
#                                             upAxisIndex=2)

# For real Time, set_RealTime 1=true or 0=false
useRealTimeSimulation = 1
if (useRealTimeSimulation):
    p.setRealTimeSimulation(1)


while 1:

    if (useRealTimeSimulation):
# p.setGravity(0, 0, -9.81)
		time.sleep(1./240.) # Time in seconds. (premade_time(0.01))

# for i in range(0,1000):
# 		# Simulation and Params
# 		p.stepSimulation()
# 		time.sleep(1./240.)
# 		# p.setTimeStep()

		# Read - Viz on  GUI
		incline_frict = p.readUserDebugParameter(inclineUI)
		r_frict = p.readUserDebugParameter(r_boxUI)
		w_frict = p.readUserDebugParameter(w_boxUI)
		# spinningFriction = p.readUserDebugParameter(spinningFrictionId)

		# Dynamics_GUI
		p.changeDynamics(incline,0, lateralFriction = 0.5)
		p.changeDynamics(r_box, -1, lateralFriction = r_frict)
		p.changeDynamics(w_box, -1, lateralFriction = w_frict)

		# Get Dynamics Info - Get Camera Viz - Write - Debug
		pos, orn = p.getBasePositionAndOrientation(r_box)
		pos1, orn1 = p.getBasePositionAndOrientation(w_box)

		r_box_velocity = p.getBaseVelocity(r_box, )
		w_box_Velocity = p.getBaseVelocity(w_box, )
		# It's for Link Description,
		info = p.getDynamicsInfo(w_box,-1)

		# getDebugVisualizerCamera											viewMatrix=[10., 10., 10., 1] or  view_matrixRPY
		k = p.getCameraImage(width=Render_width, height=Render_height, viewMatrix=view_matrix, projectionMatrix=fov_project_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
		img_rgb = cv2.cvtColor(k[2], cv2.COLOR_BGR2RGB)
		cv2.imshow('img_rgb', img_rgb )

		# Get- Viz- Write- Debug-
		# img1 = p.getCameraImage(width=128, height=128)
		# img_rgb = cv2.cvtColor(img1[2], cv2.COLOR_BGR2RGB)
		# cv2.imshow('img_rgb', img_rgb)
		cv2.waitKey(300)

		#  #if smth for discreete steps take a pic (na min pairnw oles tis eikones)
		# cap = img_path+'/img%03d.png' % snap
		# cv2.imwrite(cap, img_rgb)
		snap +=1

	# else:
	#     p.stepSimulation()

# p.disconnect(0)


# img_arr = p.getCameraImage(x, y, viewMatrix, projectionMatrix, [0, 1, 0])
