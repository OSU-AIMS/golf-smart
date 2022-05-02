

import rospy
import os
import numpy as np

from trajectory_action_client import SimpleTrajectoryActionClient




def main():
	"""
	

	"""

	# Parameters
	# joint_names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
	joint_names = rospy.get_param('controller_joint_names')
	
	# Setup Control Client
	traj_plan = SimpleTrajectoryActionClient(joint_names)

	# Load Trajectory Plan (list of joints)
	# cwd = os.path.dirname(os.path.realpath(__file__))
	# traj_plan_file_name = "sample_trajectory.csv"
	# traj_plan_file_path = open(os.path.join(cwd,traj_plan_file_name))
	# joint_array = np.genfromtxt(traj_plan_file_path, delimiter=",")
	# rospy.loginfo("Imported Array of Shape: " + str(np.shape(joint_array)))

	# Loop through Trajectory Plan
	# Note: velocities should be all 0 for the first and last trajectory point
	t = 0
	i = 0


	# print( (len(joint_array)/2)-1 )


	
	# for i, row in enumerate(joint_array):
	# 	t += 2
	# 	# rowx = np.deg2rad(row)
	# 	rowx = np.ndarray.tolist(row)
	# 	rowx = [0 if x != x else x for x in rowx]

	# 	# Set Velocity
	# 	vel = [0.05] * len(joint_names)
	# 	# vel = [0.0] * len(joint_names)
	# 	if i == 0 or i == len(joint_array)-1:
	# 		vel = [0.0] * len(joint_names)

	# 	elif i == (len(joint_array)/2):
	# 		vel = [0.0] * len(joint_array)

	# 	elif i > (len(joint_names)/2):
	# 		vel = [-1*i for i in vel]

	# 	traj_plan.add_joint_waypoint(rowx, t, vel)
	# 	rospy.logdebug("Added Waypoint #%s.  Waypoint: %s", i, rowx)
	
	# Send Plan
	traj_plan.send_trajectory()


if __name__ == '__main__':
    main()