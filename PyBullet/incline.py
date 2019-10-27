import pybullet as p
import cv2
import time
import pybullet_data
import os


# Link to imported items
# link = '/home/gpaps/PycharmProjects/Msc_Thesis/bullet3-master/data/'

link = '/home/evangeloit/Desktop/py-msc/bullet3-2.88/data'
# item_link = '/home/gpaps/PycharmProjects/Msc_Thesis/PyBullet'
item_link = '/home/evangeloit/Desktop/py-msc/Msc_Thesis/PyBullet'


# Set Environment "Constants"
# physicsClient = \
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
# planeId = p.loadURDF(link+"/plane100.urdf")


# Items Position - # https://www.andre-gaschler.com/rotationconverter/
cubeStartPos = [0, -0.3, 0.015]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
cubeStartPos1 = [0, 1.6, 0.75] 			#[0, 5.2, 2.13]
cubeStartOrientation1 = p.getQuaternionFromEuler([0, 0.5, -1.6])	#([0, 0.5, -1.6])


# Items
r_box = p.loadURDF(item_link+"/cube.urdf", cubeStartPos, cubeStartOrientation)
w_box = p.loadURDF(item_link+"/cube1.urdf", cubeStartPos1, cubeStartOrientation1)


# Import .obj-file(Incline-RigidB), Collision+Visual-> Multibody
shift = [-.5, -0.11, 0.0]
meshScale = [1, 1, 1]


# the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=item_link+"/inclinedCM.obj",
										  collisionFramePosition=shift, meshScale=meshScale)

visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=item_link+"/inclinedCM.obj",
									rgbaColor=[1, 1, 1, 0.9], specularColor=[0.4, 0.4, 0],
									visualFramePosition=shift, meshScale=meshScale)

incline = p.createMultiBody(baseMass=0, basePosition=[0, 0, 0.1],
							baseOrientation=[1, 1, 1, 1], baseInertialFramePosition=[0, 0, 0],
							baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex=visualShapeId,
							useMaximalCoordinates=True)
# Dynamics
#p.changeDynamics(incline ,-1, lateralFriction= 0.5)


# Create UI -edita
inclineUI = p.addUserDebugParameter("Incline friction", 0, 1, 0.5)
r_boxUI = p.addUserDebugParameter("R_box friction", 0, 1, 0.5)
w_boxUI = p.addUserDebugParameter("G_Box friction", 0, 1, 0.8)
# spinningFrictionId = p.addUserDebugParameter("spinning friction", 0, 1, 0.03)


# Start with wireframe ON=1 off=0,
p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)


# counter and path to save pics in each session
# img_path ='/home/gpaps/PycharmProjects/Images'
img_path ='/home/evangeloit/Desktop/py-msc/Imgs'
snap = 0
# raw_input('Press enter to continue...')

# Synthetic Camera Rendering
p.computeViewMatrix()

p.computeProjectionMatrixFOV()


# Set Camera potition -observer-
distance = 0.925	#6
yaw = -270.80 #90
height = -88.5  #40
p.resetDebugVisualizerCamera(distance, yaw, height, [0.0, 0.4625, 0.0]) #x,2,z





# for real Time, set_RealTime 1=true, 0=false
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

		# read UI
		incline_frict = p.readUserDebugParameter(inclineUI)
		r_frict = p.readUserDebugParameter(r_boxUI)
		g_frict = p.readUserDebugParameter(w_boxUI)
		# spinningFriction = p.readUserDebugParameter(spinningFrictionId)

		# Dynamics_UI
		p.changeDynamics(incline,0, lateralFriction = 0.5)
		p.changeDynamics(w_box, -1, lateralFriction = g_frict)
		p.changeDynamics(r_box, -1, lateralFriction = r_frict)

		# get
		pos, orn = p.getBasePositionAndOrientation(r_box)
		pos1, orn1 = p.getBasePositionAndOrientation(w_box)

		velocity = p.getBaseVelocity(r_box, )
		p.getBaseVelocity(w_box, )

		info = p.getDynamicsInfo(w_box,-1)

		# getDebugVisualizerCamera
		# k = p.getCameraImage(width=128, height=128, viewMatrix=[10., 10., 10., 1.])
		# img_rgb = cv2.cvtColor(k[2], cv2.COLOR_BGR2RGB)
		# cv2.imshow('img_rgb', img_rgb )

		# Get- Viz- Write- Debug-
		img1 = p.getCameraImage(width=128, height=128)
		img_rgb = cv2.cvtColor(img1[2], cv2.COLOR_BGR2RGB)
		# cv2.imshow('img_rgb', img_rgb)
		# cv2.waitKey(300)

		#  #if smth for discreete steps take a pic (na min pairnw oles tis eikones)
		# cap = img_path+'/img%03d.png' % snap
		# cv2.imwrite(cap, img_rgb)
		snap +=1

	# else:
	#     p.stepSimulation()

# p.disconnect(0)

# Projection matrix
# viewMatrix = p.computeViewMatrix(positionCamera, orientationCamera, [0, 0, 1])
# aspect = x / y;
# projectionMatrix = p.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane);
#
# img_arr = p.getCameraImage(x, y, viewMatrix, projectionMatrix, [0, 1, 0])
