import pybullet as p
import time
import math

# cid = p.connect(p.SHARED_MEMORY)
# if (cid < 0):
p.connect(p.GUI)
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setTimeStep(1. / 120.)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")
#useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)
p.loadURDF("/home/gpaps/PycharmProjects/Msc_Thesis/bullet3-master/data/plane100.urdf", useMaximalCoordinates=True)
#disable rendering during creation.
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
#disable tinyrenderer, software (CPU) renderer, we don't use it here
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

# shift = [0, -0.02, 0]
# meshScale = [0.1, 0.1, 0.1]
#the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="/home/gpaps/Documents/Objects/Collision_Tropical_Wood.obj.obj",
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0])#,
                                    # visualFramePosition=shift,
                                    # meshScale=meshScale)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="/home/gpaps/Documents/Objects/Collision_Tropical_Wood.obj.obj")#,
                                          # collisionFramePosition=shift,
                                          # meshScale=meshScale)
#
# visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
#                                     fileName="/home/gpaps/Documents/Objects/90_deg_InClined.obj",
#                                     rgbaColor=[1, 1, 1, 1],
#                                     specularColor=[0.4, .4, 0],
#                                     visualFramePosition=shift,
#                                     meshScale=meshScale)
# collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
#                                           fileName="/home/gpaps/Documents/Objects/90_deg_InClined.obj",
#                                           collisionFramePosition=shift,
#                                           meshScale=meshScale)




rangex = 5
rangey = 5
# for i in range(rangex):
#   for j in range(rangey):
#     p.createMultiBody(baseMass=1,
#                       baseInertialFramePosition=[0, 0, 0],
#                       baseCollisionShapeIndex=collisionShapeId,
#                       baseVisualShapeIndex=visualShapeId,
#                       basePosition=[((-rangex / 2) + i) * meshScale[0] * 2,
#                                     (-rangey / 2 + j) * meshScale[1] * 2, 1],
#                       useMaximalCoordinates=True)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.stopStateLogging(logId)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)


