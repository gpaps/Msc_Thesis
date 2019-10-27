import pybullet
import pybullet_data
import os

pybullet.connect(pybullet.GUI)
# without GUI: pybullet.connect(pybullet.DIRECT)

pybullet.resetSimulation()
#These support multiple objects and allow you to load entire simulation scenarios at once...
# pybullet also comes with some objects that are often useful, for example, a plane
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = pybullet.loadURDF("plane.urdf")

# this may take a while...
os.system("git clone https://github.com/ros-industrial/kuka_experimental.git")

robot = pybullet.loadURDF("kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf")

position, orientation = pybullet.getBasePositionAndOrientation(robot)
orientation
print(orientation)

joint_index = 2
_, name, joint_type, _, _, _, _, _, lower_limit, upper_limit, _, _, _ = \
pybullet.getJointInfo(robot, joint_index)
name, joint_type, lower_limit, upper_limit
print(joint_index)

joint_positions = [j[0] for j in pybullet.getJointStates(robot, range(6))]
joint_positions

world_position, world_orientation = pybullet.getLinkState(robot, 2)[:2]
world_position

pybullet.setGravity(0, 0, -9.81)   # everything should fall down
pybullet.setTimeStep(32)      # this slows everything down, but let's be accurate...  0.0001)
pybullet.setRealTimeSimulation(64)  # we want to be faster than real time :)

pybullet.setJointMotorControlArray(
    robot, range(6), pybullet.POSITION_CONTROL,
    targetPositions=[0.1] * 6)

for _ in range(10000):
    pybullet.stepSimulation()

pybullet.resetSimulation()
plane = pybullet.loadURDF("plane.urdf")
robot = pybullet.loadURDF("kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf",
                          [0, 0, 0], useFixedBase=1)  # use a fixed base!
# pybullet.setGravity(0, 0, -9.81)
# pybullet.setTimeStep(32)
# pybullet.setRealTimeSimulation(646)

pybullet.setJointMotorControlArray(
    robot, range(6), pybullet.POSITION_CONTROL,
    targetPositions=[0.1] * 6)
for _ in range(10000):
    pybullet.stepSimulation()

pybullet.disconnect()
