#!/usr/bin/env python
"""
Use pure pursuit controller to control a differential drive robot to follow a set of waypoints
"""

from world import World, Pose, Robot
import numpy as np
from time import sleep
from path_planner import PathPlanner
from controller import PurePursuit, LogUtil

# setup logging
LogUtil.set_up_logging('PurePursuit.txt')

# init world
world = World()
# timestep for world update
dt = 0.1
goal_tolerance = 0.25

# initialize planner and controller
# waypoints, goal = PathPlanner.plan(world, 10)
waypoint_list = [[1.5, 3], [2, 6], [3, 7], [4, 6], [5, 6.5], [6, 8], [8, 8], [9, 6], [6, 4], [7, 3], [8, 2.5]]
waypoints, goal = PathPlanner.create_waypoints(waypoint_list)
world.robot = Robot(Pose(waypoints[0].position, np.pi))

max_linear_velocity = 0.5
max_angular_velocity = np.pi / 3.0
look_ahead_dist = 0.1
controller = PurePursuit(waypoints, max_linear_velocity, max_angular_velocity, look_ahead_dist)

# init pygame screen for visualization
screen = world.init_screen()

while True:
    # collision testing
    if world.in_collision():
        print 'Collision'
        break

    # check if we have reached our goal
    vehicle_pose = world.robot.pose
    goal_distance = np.linalg.norm(vehicle_pose.position - goal)
    if goal_distance < goal_tolerance:
        print 'Goal Reached'
        break

    # if we want to update vehicle commands while running world
    old_pose = world.robot.pose
    min_distance = 10000.0
    op_steer = 0.0
    op_velocity = 0.0
    for i in range(1):
        world.robot.pose = old_pose
        look_ahead_dist = 0.1 + i * 0.1
        controller.setLookHeadDistance(look_ahead_dist)
        planned_linear_velocity, steer = controller.control(world.robot)
        world.robot.set_commands(planned_linear_velocity, steer)

        distance = controller.nearDistance(world.robot)
        if min_distance > distance:
            min_distance = distance
            op_steer = steer
            op_velocity = planned_linear_velocity


    print ("Optimal velo {}, steer {}".format(op_velocity, op_steer))

    # with the optimal steer & velocity
    world.robot.set_commands(op_velocity, op_steer)

    # update world
    world.update(dt)
    # draw world
    world.draw(screen, waypoints)

# pause some time to view result
sleep(2.0)
