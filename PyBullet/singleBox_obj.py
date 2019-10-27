import pybullet as p
import time
import math

p.connect(p.SHARED_MEMORY)
p.connect(p.GUI)
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setTimeStep(1./120.)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")
#useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)
p.loadURDF("/home/evangeloit/Desktop/py-msc/bullet3-2.88/data/plane100.urdf", useMaximalCoordinates=True)
# disable rendering during creation.
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
# disable tiny renderer, software (CPU) renderer, we don't use it here
# p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,0)

shift = [0,-0.02,0]
meshScale=[0.1,0.1,0.1]

rangex = 1
rangey = 2
for i in range (rangex):
	for j in range (rangey):
	# the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
		collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="/home/evangeloit/Desktop/py-msc/Blender_models/Inclined_Project/Single_Items/Tropical_Wood_Box.obj",
										  collisionFramePosition=shift,
										  meshScale=meshScale, )

		visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="/home/evangeloit/Desktop/py-msc/Blender_models/Inclined_Project/Single_Items/Tropical_Wood_Box.obj", rgbaColor=[1,1,1,1],
									specularColor=[0.4,.4,0],
									visualFramePosition=shift,
									meshScale=meshScale)

# rangex = 1
# rangey = 2
# for i in range (rangex):
# 	for j in range (rangey):
# 		p.createMultiBody(baseMass=1,baseInertialFramePosition=[0,0,0],baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex = visualShapeId, basePosition = [((-rangex/2)+i)*meshScale[0]*2,(-rangey/2+j)*meshScale[1]*2,1], useMaximalCoordinates=True)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
p.stopStateLogging(logId)
p.setGravity(0, 0, -10)

while(1): p.setRealTimeSimulation(1)
# p.disconnect(p.GUI)

