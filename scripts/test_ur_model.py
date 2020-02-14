#!/usr/bin/env python
import openravepy as orpy
import time
import numpy as np
import tf.transformations as tr

class PickPlace(object):

	def __init__(self):
		self.env = orpy.Environment()
		self.env.SetViewer('qtcoin')
#		self.env.Load('../robots/ur10_robotiq_85_gripper.robot.xml')
		self.env.Load('../worlds/cubes_task.env.xml')
		self.env.SetDefaultViewer()
		self.robot = self.env.GetRobot('robot')
		self.manipulator = self.robot.SetActiveManipulator('gripper')
		self.robot.SetActiveDOFs(self.manipulator.GetArmIndices())
		self.links = self.robot.GetLinks()
		self.box_centroid = []
		self.qgrasp = []

	def get_links(self):
		print self.links
	
	def set_env(self):
		with self.env:
			box = orpy.RaveCreateKinBody(self.env, '')
			box.SetName('box')
			box.InitFromBoxes(np.array([[0.5, 0.3, 1, 0.01, 0.04, 0.22]]), True)
			self.env.AddKinBody(box)
		self.box_centroid = box.ComputeAABB().pos()
		print self.box_centroid

	def calculate_jacobian_translation(self, link_name):
		jacobian = []
		link_idx = [l.GetName() for l in self.robot.GetLinks()].index(link_name) 
		link_origin = self.robot.GetLink(link_name).GetTransform()[:3,3]
		np.set_printoptions(precision=6, suppress=True)
		jacobian_translation = self.robot.ComputeJacobianTranslation(link_idx, link_origin)
		jacobian_angular = self.robot.ComputeJacobianAxisAngle(link_idx)
		jacobian.append(jacobian_translation)
		jacobian.append(jacobian_angular)
		return jacobian

	def ik_calculation(self):
		ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=orpy.IkParameterization.Type.Transform6D)
		if not ikmodel.load():
			print "ik model loading failed"
  			ikmodel.autogenerate()
			print "auto generating ik model"
		Tgrasp = tr.quaternion_matrix([ 0.5,  0.5,  0.5, -0.5])
		Tgrasp[:3,3] = self.box_centroid
		print "finding the ik solutions"
		solutions = self.manipulator.FindIKSolutions(Tgrasp, 0)
		self.qgrasp = solutions[0]
		print solutions
		for solution in solutions:
                	self.robot.SetActiveDOFValues(solution)
			raw_input("Press to see the next configuration")

	def path_planning(self):
		planner = orpy.RaveCreatePlanner(self.env, 'birrt') # Using bidirectional RRT
		params = orpy.Planner.PlannerParameters()
		params.SetRobotActiveJoints(self.robot)
		params.SetGoalConfig(self.qgrasp)
		print "goal is: ", self.qgrasp
		params.SetPostProcessing('ParabolicSmoother', '<_nmaxiterations>40</_nmaxiterations>')
		planner.InitPlan(self.robot, params)
		# Plan a trajectory
		traj = orpy.RaveCreateTrajectory(self.env, '')
		planner.PlanPath(traj)
		# Execute the trajectory
		controller = self.robot.GetController()
		controller.SetPath(traj)

if __name__ == "__main__":
	scene = PickPlace()
	scene.set_env()
	scene.ik_calculation()
	jacobian = scene.calculate_jacobian_translation('robotiq_85_base_link')
	print "translation jacobian: ", jacobian[0]
	print "angular jacobian: ", jacobian[1]
	#scene.path_planning()
	raw_input("Press any key to exit")

