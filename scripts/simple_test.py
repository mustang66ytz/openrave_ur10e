#!/usr/bin/env python
import openravepy as orpy
import time
import numpy as np
import tf.transformations as tr

env = orpy.Environment()
env.SetViewer('qtcoin')
env.Load('../worlds/cubes_task.env.xml')
env.SetDefaultViewer()
robot = env.GetRobot('robot')
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())
links = robot.GetLinks()


ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=orpy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
	print "ik model loading failed"
	ikmodel.autogenerate()
			
Tgrasp = tr.quaternion_matrix([ 0.5,  0.5,  0.5, -0.5])
Tgrasp[:3,3] = [0.5, 0.3, 1]
print "finding the ik solutions"
solutions = manipulator.FindIKSolutions(Tgrasp, 0)
qgrasp = solutions[0]
print solutions
for solution in solutions:
	robot.SetActiveDOFValues(solution)
	raw_input("Press to see the next configuration")

