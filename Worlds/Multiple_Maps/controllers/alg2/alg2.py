# alg2.py

"""sensor_rewrite controller."""

import numpy as np
from initialization import *
import math

# leave all comments for readability purposes
# leave all print statements for debugging purposes
# create a new comment if you make changes

if __name__ == "__main__":
    TIME_STEP = 32
    
    # MODIFIED: Receive the trail group handle from init_robot
    robot, goal_pos, start_pos, trail_group_children_field = init_robot(time_step=TIME_STEP)
    
    print("Robot Initialized")
    init_robot_state(in_pos=[0,0,0], in_omega=[0,0])
    prev = ""
    m_line = calculate_slope(goal_pos[0], goal_pos[1], start_pos[0], start_pos[1])
    state = 'start'
    robot_speed = 3
    hit_point = []
    leave_point = []
    next_turn = 'left'
    alt_turn = False
    turnaround_point = []
    turnaround_inprogress = False
    turn_to_angle = 0
    starttime = robot.getTime()
    
    # NEW: Counter for dropping trail markers
    trail_counter = 0

    # robot loop
    while robot.step(TIME_STEP) != -1:
        gps_values, compass_val, encoder_value, ir_value, imu_yaw = read_sensors_values()

        # logic to drop trail sphere
        trail_counter += 1
        if trail_counter % 30 == 0 and trail_group_children_field is not None:
            pos = gps_values
            # Create a pink/purple, semi-transparent sphere
            sphere_string = f"""
                Transform {{
                    translation {pos[0]} {pos[1]} 0.01
                    children [
                        Shape {{
                            appearance PBRAppearance {{
                                baseColor 1 0.2 0.8
                                transparency 0.5
                                roughness 1
                            }}
                            geometry Sphere {{ radius 0.015 }}
                        }}
                    ]
                }}
            """
            trail_group_children_field.importMFNodeFromString(-1, sphere_string)
        
        front_ir_values = ir_value[0], ir_value[7]
        right_ir_values = ir_value[1], ir_value[2]
        left_ir_values = ir_value[5], ir_value[6]
        update_robot_state()

        if state == 'start':
            print("Running start state")
            prev = state 
            state = 'align_robot_heading'

        # checks to see if robot is aligned. if aligned to the direction of the goal position, start moving
        elif state == 'align_robot_heading':
            print("Running align robot heading state")
            if alt_turn == True:
                is_aligned = align_to_M(turn_to_angle, imu_yaw)
            else:
                is_aligned = align_to_M(calculate_target_angle(gps_values, goal_pos), imu_yaw)
                # print("target angle: ", calculate_target_angle(gps_values, goal_pos))
                # print("imu yaw: ", imu_yaw)
            if is_aligned: 
                prev = state
                if alt_turn == True:
                    state = 'wall_following'
                else:
                    state = 'move_to_goal'
                alt_turn = False

        elif state == 'move_to_goal':
            print("Running move to goal state")
            update_motor_speed(input_omega=[robot_speed, robot_speed])
            turnaround_inprogress = False
            if (calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])<0.17): # at the goal
                state = 'end'
            elif (front_ir_values[0] + front_ir_values[1]) / 2 > 800: # object detected, wall-follow
                prev = state
                state = 'wall_following'
                hit_point.append([gps_values[0], gps_values[1]])

        elif state == 'wall_following': # follows perimeter of obstacle
            print("Running wall_following state")
            left_wall = left_ir_values[0] > 80
            front_wall = ((front_ir_values[0] + front_ir_values[1]) / 2) > 80
            right_wall = right_ir_values[1] > 80
            print("Left wall: ", left_ir_values[0])
            print("Front wall: ", front_ir_values[0], " and ", front_ir_values[1])
            print("Right wall: ", right_ir_values[1])

            angle_to_goal = calculate_target_angle(gps_values, goal_pos)
            open_path = is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall)
            prev_distance = calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_pos[0], goal_pos[1])
            curr_distance = calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])
            #print("Previous distance: ", prev_distance)
            #print("Current Distance: ", curr_distance)


            if front_wall: # obstacle in front
                print("Running front wall")
                if next_turn == 'left':
                    update_motor_speed(input_omega=[-1*robot_speed, robot_speed])
                else:
                    update_motor_speed(input_omega=[robot_speed, -1*robot_speed])
            
            elif (right_wall ^ left_wall): # wall-follow based on wall
                print("Running Right or Left Wall") 
                update_motor_speed(input_omega=[robot_speed, robot_speed])
                if open_path and turnaround_inprogress:
                    # print("turnaround")
                    hptg = calculate_euclidean_distance(turnaround_point[0], turnaround_point[1], goal_pos[0], goal_pos[1])
                    rtg = calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])
                    if (hptg > rtg):
                        leave_point.append([gps_values[0], gps_values[1]])
                        prev = state
                        state = 'align_robot_heading'
                if open_path and curr_distance < prev_distance and turnaround_inprogress == False:
                    # print ("leaving wall")
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'
                if prev == "wall_following":
                    # print ("finding hit points")
                    for i, hitX in enumerate(hit_point):
                        hp_togoal = calculate_euclidean_distance(hitX[0], hitX[1], goal_pos[0], goal_pos[1])
                        robot_togoal = calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])
                        hp_to_robot = calculate_euclidean_distance(hitX[0], hitX[1], gps_values[0], gps_values[1])
                        safety = False
                        if len(leave_point) == 0:
                            safety = True
                        else:
                            lhp_togoal = calculate_euclidean_distance(leave_point[-1][0], leave_point[-1][1], goal_pos[0], goal_pos[1])
                            if robot_togoal > lhp_togoal:
                                safety = True
                        if (abs(robot_togoal - hp_togoal) < 0.08) and (hp_to_robot < 0.08) and (safety):
                            turnaround_point = hitX
                            turnaround_inprogress = True
                            if next_turn == 'left':
                                next_turn = 'right'
                            else:
                                next_turn = 'left'
                            turn_to_angle = normalize_angle(imu_yaw-180)
                            alt_turn = True
                            prev = state
                            state = 'align_robot_heading'
            else: # for getting around corners
                print("Running else")
                if next_turn == 'left':
                    update_motor_speed(input_omega=[robot_speed, robot_speed/8])
                elif next_turn == 'right':
                    update_motor_speed(input_omega=[robot_speed/8, robot_speed])
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