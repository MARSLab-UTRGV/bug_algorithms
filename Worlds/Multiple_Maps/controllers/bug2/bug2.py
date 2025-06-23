# bug2.py

"""sensor_rewrite controller."""

import numpy as np
import time
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
    if angle_to_target < 0:
        angle_to_target += 360
    elif angle_to_target > 360:
        angle_to_target -= 360 
    #print("normalised angle: ", angle_to_target)
    return angle_to_target


# boolean function: rotates robot if not pointed to M
def align_to_M(target_angle, yaw_angle, threshold = 2):
    ts = 1  # turning speed
    turn = 'left'

    difference = target_angle - yaw_angle

    halfway = target_angle - 180
    #print("Halfway not normalized: ", halfway)
    if halfway < 0:
        halfway += 360
    
    #print("Target Angle: ", target_angle)
    #print("Yaw Angle: ", yaw_angle)
    #print("Halfway: ", halfway)
    #print("Difference: ", difference)

    if target_angle <= 180 and yaw_angle <= 180:
        #print("Both angles less than 180")
        if target_angle < yaw_angle:
            turn = 'right'
            #print("Target angle is less than yaw. Turn right")
        else: 
            turn = 'left'
            #print("Target angle is more than yaw. Turn left")
    elif (target_angle <= 360 and yaw_angle <= 360) and (target_angle >=180 and yaw_angle >= 180):
        #print("Both angles greater than 180 and less than 360")
        if target_angle < yaw_angle:
            turn = 'right'
            #print("Target angle is less than yaw. Turn right")
        else: 
            turn = 'left'
            #print("Target angle is more than yaw. Turn left")
    else:
        if yaw_angle > halfway:
            turn = 'left'
            #print("Yaw is greater than halfway. Turn left")
        else: 
            turn = 'right'
            #print("Yaw is less than halfway. Turn right")

    if abs(difference) < threshold:
        #print("Aligned")
        return True
    else:
        if turn == 'left':
            #print("Turning left")
            update_motor_speed(input_omega=[-ts, ts/20])
        else: 
            #print("Turning right")
            update_motor_speed(input_omega=[ts/20, -ts])
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
    #print("Slope: ", (y1-y2)/(x1-x2))
    return((y1-y2)/(x1-x2))

# boolean function: Checks if robot is on m_line
# write in the m-line to compare instead of calculating the slope every time
def is_on_M_line(currX, currY, goalX, goalY, m_line, threshold=0.1):
    slope = calculate_slope(currX, currY, goalX, goalY)
    #print("Running M-Line Function")
    #print("Slope: ", slope)
    #print("m_line: ", m_line)
    #print("Difference: ", abs(m_line - slope))
    return abs(m_line - slope) < threshold

def is_open(yaw, atg, right, left, front):
    #print("Checking if open")
    #print("Right Wall: ", right)
    #print("Left Wall: ", left)
    #print("Front Wall: ", front)
    #print("yaw = ", yaw)
    #print("atg = ", atg)
    normalized_right = yaw - 135
    normalized_left = yaw + 135
    normalized_frontone = yaw - 15
    normalized_fronttwo = yaw + 15
    if normalized_left > 360:
        normalized_left -= 360
    if normalized_right < 0:
        normalized_right += 360
    if normalized_frontone < 0:
        normalized_frontone += 360
    if normalized_fronttwo > 360:
        normalized_fronttwo -= 360
    front_angle = [normalized_frontone, normalized_fronttwo]
    right_angle = [normalized_right, front_angle[0]]
    left_angle = [front_angle[1], normalized_left]

    if front == False:
        if (atg >= front_angle[0] and atg <= front_angle[1]):
            return True      
        if (front_angle[0] > front_angle[1]) and (atg >= (front_angle[0]-360) and atg <= front_angle[1]):
            return True
    if right == False:
        if (atg >= right_angle[0] and atg <= right_angle[1]):
            return True
        if (right_angle[0] > right_angle[1]) and (atg >= (right_angle[0]-360) and atg <= right_angle[1]):
            return True
    if left == False:
        if (atg >= left_angle[0] and atg < left_angle[1]):
            return True
        if (left_angle[0] > left_angle[1]) and (atg >= (left_angle[0]) and atg <= left_angle[1]+360):
            return True
    return False

if __name__ == "__main__":
    TIME_STEP = 32
    
    # Receive the trail group handle from the modified init_robot function
    robot, goal_pos, start_pos, trail_group_children_field = init_robot(time_step=TIME_STEP)
    
    print("Robot Initialized")
    init_robot_state(in_pos=[0,0,0], in_omega=[0,0])
    prev = ""
    m_line = calculate_slope(goal_pos[0], goal_pos[1], start_pos[0], start_pos[1])
    state = 'start'
    robot_speed = 3
    hit_point = []
    leave_point = []
    starttime = robot.getTime()
    
    # Counter for dropping trail markers
    trail_counter = 0

    while robot.step(TIME_STEP) != -1:
        
        gps_values, compass_val, encoder_value, ir_value, imu_yaw = read_sensors_values()
        
        # --- LOGIC TO DROP A TRAIL SPHERE ---
        trail_counter += 1
        # Drop a sphere every 15 simulation steps to create a trail
        if trail_counter % 15 == 0 and trail_group_children_field is not None:
            pos = gps_values
            
            # Create a string defining a blue, semi-transparent sphere
            sphere_string = f"""
                Transform {{
                    translation {pos[0]} {pos[1]} 0.01
                    children [
                        Shape {{
                            appearance PBRAppearance {{
                                baseColor 0.2 0.5 1 
                                transparency 0.5
                                roughness 1
                            }}
                            geometry Sphere {{ radius 0.015 }}
                        }}
                    ]
                }}
            """
            # Add the new sphere to the trail group in the world
            trail_group_children_field.importMFNodeFromString(-1, sphere_string)
        # --- END OF TRAIL LOGIC ---

        front_ir_values = ir_value[0], ir_value[7]
        right_ir_values = ir_value[1], ir_value[2]
        left_ir_values = ir_value[5], ir_value[6]
        update_robot_state()

        # --- Your original state machine logic (unchanged) ---
        if state == 'start':
            print("Running start state")
            prev = state 
            state = 'align_robot_heading'
        elif state == 'align_robot_heading':
            print("Running align robot heading state")
            is_aligned = align_to_M(calculate_target_angle(gps_values, goal_pos), imu_yaw)
            if is_aligned: 
                prev = state
                state = 'move_to_goal'
            calculate_slope(goal_pos[0], goal_pos[1], gps_values[0], gps_values[1])
        elif state == 'move_to_goal':
            print("Running move to goal state")
            update_motor_speed(input_omega=[robot_speed, robot_speed])
            difference = abs(front_ir_values[1] - front_ir_values[0])
            if (calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])<0.17):
                state = 'end'
            elif (front_ir_values[0] + front_ir_values[1]) / 2 > 800:
                prev = state
                state = 'wall_following'
                hit_point.append([gps_values[0], gps_values[1]])
        elif state == 'wall_following':
            print("Running wall_following state")
            left_wall = left_ir_values[0] > 80
            front_wall = ((front_ir_values[0] + front_ir_values[1]) / 2) > 80
            right_wall = right_ir_values[1] > 80
            angle_to_goal = calculate_target_angle(gps_values, goal_pos)
            if front_wall: 
                print("Running front wall")
                update_motor_speed(input_omega=[-1*robot_speed, robot_speed])
            elif (is_on_M_line(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1], m_line)) and prev == 'wall_following':
                print("Running M_line")
                open_path = is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall)
                if (calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1]) < calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_pos[0], goal_pos[1]) and open_path):
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'
            elif right_wall:
                print("Running Right Wall") 
                update_motor_speed(input_omega=[robot_speed, robot_speed])
            else:
                print("Running else")
                update_motor_speed(input_omega=[robot_speed, robot_speed/2])
                prev = state
        elif state == 'end':
            print("Running end state")
            update_motor_speed(input_omega=[0, 0, 0])
            endtime = robot.getTime()
            elapsedtime = endtime - starttime
            print(f"Time taken to reach goal: {elapsedtime:.2f} seconds")
            break
        elif(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])< 0.1):
            state = 'end'
            break
        
    pass