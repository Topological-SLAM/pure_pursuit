#!/usr/bin/env python
"""
Use pure pursuit controller to control a differential drive robot to follow a set of waypoints
"""

from world import World, Pose, Robot
import numpy as np
from time import sleep
from path_planner import PathPlanner
from controller import PurePursuit, LogUtil

import csv
csvF = open('data_waypoint.csv', 'r')
reader = csv.reader(csvF)
waypoint_list = []
for item in reader:
    item =[float(a) for a in item]
    waypoint_list.append(item)

csvF.close()

# setup logging
LogUtil.set_up_logging('PurePursuit.txt')

# init world
world = World()
# timestep for world update
dt = 0.4
goal_tolerance = 0.25

# initialize planner and controller
# waypoints, goal = PathPlanner.plan(world, 10)
waypoint_list = [[1.5, 3], [2, 6], [3, 7], [4, 6], [5, 6.5], [6, 8], [8, 8], [9, 6], [6, 4], [7, 3], [8, 2.5]]
waypoints, goal = PathPlanner.create_waypoints(waypoint_list)
world.robot = Robot(Pose(waypoints[0].position, np.pi))

max_linear_velocity = 0.1
max_angular_velocity = np.pi / 3.0
look_ahead_dist = 0.1
controller = PurePursuit(waypoints, max_linear_velocity, max_angular_velocity, look_ahead_dist)

# init pygame screen for visualization
screen = world.init_screen()
sum_distance = 0.0
step_count = 0

sum_index = np.zeros(10).astype('float')

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
    old_position, old_heading = world.robot.pose.getPose()
    min_distance = 10000.0
    op_steer = 0.0
    op_velocity = 0.0

    print ("")
    p, h = world.robot.pose.getPose()
    print (p,h)
    for i in range(1):
        world.robot.pose.setPose(old_position, old_heading)
        look_ahead_dist = 0.02 + i * 0.1
        print (look_ahead_dist)
        controller.setLookHeadDistance(look_ahead_dist)
        planned_linear_velocity, steer = controller.control(world.robot)
        print (steer)

        # update robot
        world.robot.set_commands(planned_linear_velocity, steer)
        world.update(dt)

        distance = controller.nearDistance(world.robot)
        if min_distance > distance:
            print ("fall")
            min_distance = distance
            opt_position, opt_heading = world.robot.pose.getPose()
            opt_index = i

        
    sum_index[opt_index] += 1
    print ("opt index {}".format(opt_index))
    # with the optimal steer & velocity
    world.robot.pose.setPose(opt_position, opt_heading)
    #p, h = world.robot.pose.getPose()
    #print (p,h)

    distance = controller.nearDistance(world.robot)
    sum_distance += distance
    step_count += 1

    # draw world
    world.draw(screen, waypoints)

# caculate the index 
for i in range(len(sum_index)):
    print (sum_index[i])


# pause some time to view result
sleep(2.0)
print ("Sum of distance is {}".format(sum_distance/step_count))
