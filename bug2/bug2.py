"""sensor_rewrite controller."""

import numpy as np
from initialization import *
import math


def calculate_target_angle(robot_pos, goal_pos):
    #the goal will be calculated based on 0 degrees started at north going clockwise
    #the robot will be calculated based on 0 degrees started at east going counter clockwise
    # this function calculates the goal angle and then normalizes it on a 0-360 degrees
    # then it will negate the angle so it will be based on the counter clockwise
    # then add 90 to start 0 degrees from east, and add 360 to get an angle betweeen 0-360
    dx = goal_pos[0] - robot_pos[0]
    dy = goal_pos[1] - robot_pos[1]
    angle_to_target = math.degrees(math.atan2(dx, dy))
    #print("raw angle: ", angle_to_target)
    if angle_to_target < 0:
        angle_to_target += 360
    elif angle_to_target > 360:
        angle_to_target -= 360
    angle_to_target *= -1
    angle_to_target += 450
    #print("normalised angle: ", angle_to_target)
    return angle_to_target


# boolean function: rotates robot if not pointed to M
def align_to_M(target_angle, yaw_angle, threshold = 3):
    ts = 1  # turning speed
    turn = 'left'
    difference = target_angle - yaw_angle
    print("yaw angle: ", yaw_angle)
    print("Target Angle: ", target_angle)
    print("Difference", difference)


    if(difference < 0):
        turn = 'right'

    if  abs(difference) < threshold:
        print("true")
        return True
    else:
        if turn == 'right':
            update_motor_speed(input_omega=[ts/20, -ts])
            print("right turning")
        else:
            update_motor_speed(input_omega=[-ts, ts/20])
            print("left turning")
        return False

# returns distance between two given points
def calculate_euclidean_distance(x1, y1, x2, y2):
    # print("Euclidean Distance: ", math.sqrt((x1-x2)**2 + (y1-y2)**2))
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

# returns slope of the m-line between goal and start
def calculate_slope(x1, y1, x2, y2):
    # print("Y: ", (y1-y2))
    # print("X: ", (x1-x2))
    if (x1-x2) == 0:
        return x1
    print("Slope: ", (y1-y2)/(x1-x2))
    return((y1-y2)/(x1-x2))

# boolean function: Checks if robot is on m_line
# write in the m-line to compare instead of calculating the slope every time
def is_on_M_line(currX, currY, goalX, goalY, m_line, threshold=1):
    slope = calculate_slope(currX, currY, goalX, goalY)
    # print("Slope: ", slope)
    # print("m_line: ", m_line)
    # print("Difference: ", m_line - slope)
    return abs(m_line - slope) < threshold

if __name__ == "__main__":
    # initialization of robot
    TIME_STEP = 32
    robot, goal_pos, start_pos = init_robot(time_step=TIME_STEP)
    print("Robot Initialized")
    init_robot_state(in_pos=[0,0,0], in_omega=[0,0])
    prev = ""

    # calculate m-line
    m_line = calculate_slope(goal_pos[0], goal_pos[1], start_pos[0], start_pos[1])
    # define local variables
    state = 'start'
    robot_speed = 3
    forward_left_speeds = [robot_speed, robot_speed]
    far_from_wall_counter = 0
    close_to_wall_counter = 0
    hit_point = []      # x, y
    leave_point = []    # x, y

    # robot loop
    while robot.step(TIME_STEP) != -1:

        # trash variable for unused sonar. can be taken out of initialization.py
        gps_values, compass_val, encoder_value, ir_value, imu_yaw = read_sensors_values()
        #print("Sensor Read Complete")
        # print("GPS Values: ", gps_values[0], gps_values[1])
        front_ir_values = ir_value[0], ir_value[7]
        right_ir_values = ir_value[1], ir_value[2]
        left_ir_values = ir_value[5], ir_value[6]

        update_robot_state()
        # print("Current: ", state)
        # print("Previous: ", prev)

        # checks if on M-line before start. might need to be updated to
        # calculate and save m-line points instead.
        # function also works when if statement is taken out.
        if state == 'start':
            print("Running start state")
            #if(is_on_M_line(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1], m_line)):
            prev = state 
            state = 'align_robot_heading'
            

        # checks to see if robot is aligned. if align, start moving
        elif state == 'align_robot_heading':
            print("Running align robot heading state")
            is_aligned = align_to_M(calculate_target_angle(gps_values, goal_pos), imu_yaw)#need to set the goal here for the object to orient itself to 
            if is_aligned: 
                prev = state
                state = 'move_to_goal'
            calculate_slope(goal_pos[0], goal_pos[1], gps_values[0], gps_values[1])

        # updates robot position, moving along the m-line
        # if at goal, stop; if front finds obstacle, wall follow + save hit points
        # maybe calculate slope for m-line and match for alignment and movement?
        elif state == 'move_to_goal':
            print("Running move to goal state")
            update_motor_speed(input_omega=[robot_speed, robot_speed])
            difference = abs(front_ir_values[1] - front_ir_values[0])
            # print("Difference: ", difference)
            # print("Front_ir_values: ", front_ir_values[0], " ", front_ir_values[1])
            if (calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])<0.17):
                state = 'end'
            elif (front_ir_values[0] + front_ir_values[1]) / 2 > 800:
                prev = state
                state = 'wall_following'
                hit_point.append([gps_values[0], gps_values[1]])
                    
            
        # follows perimeter of obstacle.
        elif state == 'wall_following':
            print("Running wall_following state")
            # print(prev)
            left_wall = left_ir_values[0] > 80
            front_wall = ((front_ir_values[0] + front_ir_values[1]) / 2) > 80
            right_wall = right_ir_values[1] > 80
            # print("Left Values: ", left_ir_values)
            # print("Front Values: ", front_ir_values)
            # print("Right Values: ", right_ir_values)
            # print("Left Wall: ", left_wall)
            # print("Front Wall: ", front_wall)
            # print("Right Wall: ", right_wall)

            # found wall in front, default turn to left
            if front_wall: 
                print("Running front wall")
                update_motor_speed(input_omega=[-1*robot_speed, robot_speed])

            # is on m-line after wall following, creates leave point and aligns to goal again
            elif (is_on_M_line(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1], m_line)) and prev == 'wall_following':
                # print("Running M_line")
                # print(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_postition[0], goal_postition[1]) < calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_postition[0], goal_postition[1]))
                if calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1]) < calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_pos[0], goal_pos[1]):
                    # print("Running E-Distance")
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'

            # following wall on the right
            elif right_wall:
                print("Running Right Wall") 
                update_motor_speed(input_omega=[robot_speed, robot_speed])
                #if calculate_euclidean_distance(gps_values[0], gps_values[1], hit_point[-1][0], hit_point[-1][0]) > 0.7:
                #    print("Prev Changed 1")
                #    prev = state

            # if too far from the wall
            else:
                print("Running else")
                update_motor_speed(input_omega=[robot_speed, robot_speed/2])
                # print("Prev Changed 2")
                prev = state

        elif state == 'end':
            print("Running end state")
            update_motor_speed(input_omega=[0, 0, 0]) #end
                
        elif(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])< 0.1):
            state = 'end'
        
    pass
            
                    