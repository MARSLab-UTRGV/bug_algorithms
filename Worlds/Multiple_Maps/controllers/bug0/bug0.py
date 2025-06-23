# bug0.py

"""sensor_rewrite controller."""

import numpy as np
import time
from initialization import *
import math

def calculate_target_angle(robot_pos, goal_pos):
    dx = goal_pos[0] - robot_pos[0]
    dy = goal_pos[1] - robot_pos[1]
    angle_to_target = math.degrees(math.atan2(dx, dy))
    if angle_to_target < 0: angle_to_target += 360
    elif angle_to_target > 360: angle_to_target -= 360
    angle_to_target *= -1
    angle_to_target += 450
    if angle_to_target < 0: angle_to_target += 360
    elif angle_to_target > 360: angle_to_target -= 360 
    return angle_to_target

def align_to_M(target_angle, yaw_angle, threshold = 2):
    ts = 1
    turn = 'left'
    difference = target_angle - yaw_angle
    halfway = target_angle - 180
    if halfway < 0: halfway += 360
    if target_angle <= 180 and yaw_angle <= 180:
        if target_angle < yaw_angle: turn = 'right'
        else: turn = 'left'
    elif (target_angle <= 360 and yaw_angle <= 360) and (target_angle >=180 and yaw_angle >= 180):
        if target_angle < yaw_angle: turn = 'right'
        else: turn = 'left'
    else:
        if yaw_angle > halfway: turn = 'left'
        else: turn = 'right'
    if abs(difference) < threshold: return True
    else:
        if turn == 'left': update_motor_speed(input_omega=[-ts, ts/20])
        else: update_motor_speed(input_omega=[ts/20, -ts])
        return False

def calculate_euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def calculate_slope(x1, y1, x2, y2):
    if (x1-x2) == 0: return x1
    return((y1-y2)/(x1-x2))

def is_on_M_line(currX, currY, goalX, goalY, m_line, threshold=2):
    slope = calculate_slope(currX, currY, goalX, goalY)
    return abs(m_line - slope) < threshold

def is_open(yaw, atg, right, left, front):
    print("Checking if open")
    normalized_right = yaw - 90; normalized_left = yaw + 90
    normalized_frontone = yaw - 15; normalized_fronttwo = yaw + 15
    if normalized_left > 360: normalized_left -= 360
    if normalized_right < 0: normalized_right += 360
    if normalized_frontone < 0: normalized_frontone += 360
    if normalized_fronttwo > 360: normalized_fronttwo -= 360
    front_angle = [normalized_frontone, normalized_fronttwo]
    right_angle = [normalized_right, front_angle[0]]
    left_angle = [front_angle[1], normalized_left]
    if front == False:
        if (atg >= front_angle[0] and atg <= front_angle[1]): return True      
        if (front_angle[0] > front_angle[1]) and (atg >= (front_angle[0]-360) and atg <= front_angle[1]): return True
    if right == False:
        if (atg >= right_angle[0] and atg <= right_angle[1]): return True
        if (right_angle[0] > right_angle[1]) and (atg >= (right_angle[0]-360) and atg <= right_angle[1]): return True
    if left == False:
        if (atg >= left_angle[0] and atg < left_angle[1]): return True
        if (left_angle[0] > left_angle[1]) and (atg >= (left_angle[0]) and atg <= left_angle[1]+360): return True
    return False

if __name__ == "__main__":
    TIME_STEP = 32
    
    # MODIFIED: This line now correctly receives 4 values
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

    # NEW: Counter for dropping trail markers
    trail_counter = 0

    while robot.step(TIME_STEP) != -1:
        
        gps_values, compass_val, encoder_value, ir_value, imu_yaw = read_sensors_values()

        # --- NEW: LOGIC TO DROP A TRAIL SPHERE ---
        trail_counter += 1
        if trail_counter % 15 == 0 and trail_group_children_field is not None:
            pos = gps_values
            # Create a yellow, semi-transparent sphere
            sphere_string = f"""
                Transform {{
                    translation {pos[0]} {pos[1]} 0.01
                    children [
                        Shape {{
                            appearance PBRAppearance {{
                                baseColor 1 0.9 0.2
                                transparency 0.5
                                roughness 1
                            }}
                            geometry Sphere {{ radius 0.015 }}
                        }}
                    ]
                }}
            """
            trail_group_children_field.importMFNodeFromString(-1, sphere_string)
        # --- END OF NEW TRAIL LOGIC ---

        front_ir_values = ir_value[0], ir_value[7]
        right_ir_values = ir_value[1], ir_value[2]
        left_ir_values = ir_value[5], ir_value[6]
        update_robot_state()

        if state == 'start':
            prev = state 
            state = 'align_robot_heading'
        elif state == 'align_robot_heading':
            is_aligned = align_to_M(calculate_target_angle(gps_values, goal_pos), imu_yaw)
            if is_aligned: 
                prev = state
                state = 'move_to_goal'
            calculate_slope(goal_pos[0], goal_pos[1], gps_values[0], gps_values[1])
        elif state == 'move_to_goal':
            update_motor_speed(input_omega=[robot_speed, robot_speed])
            if (calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])<0.17):
                state = 'end'
            elif (front_ir_values[0] + front_ir_values[1]) / 2 > 800:
                prev = state
                state = 'wall_following'
                hit_point.append([gps_values[0], gps_values[1]])
        elif state == 'wall_following':
            left_wall = left_ir_values[0] > 80
            front_wall = ((front_ir_values[0] + front_ir_values[1]) / 2) > 80
            right_wall = right_ir_values[1] > 80
            angle_to_goal = calculate_target_angle(gps_values, goal_pos)
            open_path = is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall)
            prev_distance = calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_pos[0], goal_pos[1])
            curr_distance = calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])
            if front_wall: 
                update_motor_speed(input_omega=[-1*robot_speed, robot_speed])
            elif right_wall ^ left_wall:
                update_motor_speed(input_omega=[robot_speed, robot_speed])
                if right_wall: print("touching wall on right")
                elif left_wall: print("touching wall on left")
                if open_path and curr_distance < prev_distance:
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'
            else:
                update_motor_speed(input_omega=[robot_speed, robot_speed/10])
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