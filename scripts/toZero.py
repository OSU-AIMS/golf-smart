#!/usr/bin/env python


from trajectory_action_client import SimpleTrajectoryActionClient

joint_names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']

# Go To Zero
traj_plan_start = SimpleTrajectoryActionClient(joint_names)
traj_plan_start.add_joint_waypoint([0,0,0,0,0,0],3,[0,0,0,0,0,0])
traj_plan_start.send_trajectory()

# EOF