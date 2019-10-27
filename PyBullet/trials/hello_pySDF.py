import pybullet as p
import time
import pybullet_data

string = "/home/gpaps/PycharmProjects/Msc_Thesis/PyBullet/trials/"
stringSDF = "/home/gpaps/Documents/Gazebo_items/"

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

p.setGravity(0, 0, -9.8)

boxId = p.loadSDF(string + "table.sdf")
boxId1 = p.loadSDF(string + "box1.sdf")

for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()




