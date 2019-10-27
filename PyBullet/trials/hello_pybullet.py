import pybullet as p
import time
import pybullet_data

string = "/home/gpaps/PycharmProjects/Msc_Thesis/PyBullet/trials/"
stringURDF = "/home/gpaps/Documents/Objects/urdf/"

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

planeId = p.loadURDF("plane100.urdf")

p.setGravity(0, 0, -9.81)

# obj1
cubeStartPos = [0.5, 0.5, 0]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
# boxId = p.loadURDF("/table/table.urdf", cubeStartPos, cubeStartOrientation)
boxId = p.loadURDF("/trials/table_edit.urdf", cubeStartPos, cubeStartOrientation)

# obj2
cubeStartPos1 = [0.5, 0.5, 2]
cubeStartOrientation1 = p.getQuaternionFromEuler([0, 0, 0])
boxId1 = p.loadURDF("/trials/block_edit.urdf", cubeStartPos1, cubeStartOrientation1)

for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
cubePos1, cubeOrn1 = p.getBasePositionAndOrientation(boxId1)

print(cubePos, cubeOrn)
print(cubePos1, cubeOrn1)


p.disconnect()





