import pybullet as p
import time

p.connect(p.GUI)  # Physics Client (Stand alone)
p.createCollisionShape(p.GEOM_PLANE)

box1 = p.createCollisionShape(p.GEOM_BOX)#, halfExtents=[1, 1, 1], )

boxid = p.createCollisionShape(p.GEOM_MESH, 'Collision_Tropical_Wood.obj')

mass = 1


visualShapeId = -1

p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)


while(1):
  keys = p.getKeyboardEvents()
  print(keys)

  time.sleep(0.01)



