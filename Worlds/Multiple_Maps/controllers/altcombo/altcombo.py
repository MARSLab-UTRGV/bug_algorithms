"""sensor_rewrite controller."""

import numpy as np
from initialization import *
import math

# leave all comments for readability purposes
# leave all print statements for debugging purposes
# create a new comment if you make changes

if __name__ == "__main__":
    TIME_STEP = 32
    robot, goal_pos, start_pos, trail_group_children_field = init_robot(time_step=TIME_STEP)
    
    print("Robot Initialized")
    init_robot_state(in_pos=[0,0,0], in_omega=[0,0])
    prev = ""
    wf_state = ""
    wf_prev = ""
    m_line = calculate_slope(goal_pos[0], goal_pos[1], start_pos[0], start_pos[1]) # calculate m-line
    original_m_line = m_line
    temp_m_line = None
    b = m_line_b(start_pos[0], start_pos[1], m_line)
    state = 'start'
    robot_speed = 3
    corner_turn_speed = 12
    hit_point = []      # x, y
    leave_point = []    # x, y
    turn_direction = 'CW'
    starttime = robot.getTime()
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
        on_m_line = is_on_M_line(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1], m_line)
        on_temp_line = False
        if temp_m_line != None:
            on_temp_line = is_on_M_line(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1], temp_m_line)
        update_robot_state()

        
        if state == 'start':
            prev = state 
            state = 'align_robot_heading'
            
        # checks to see if robot is aligned. if aligned to the direction of the goal position, start moving
        elif state == 'align_robot_heading':
            print("Running alignment")
            is_aligned = align_to_M(calculate_target_angle(gps_values, goal_pos), imu_yaw)
            # print("target angle: ", calculate_target_angle(gps_values, goal_pos))
            # print("imu yaw: ", imu_yaw)
            if is_aligned: 
                prev = state
                state = 'move_to_goal'

        elif state == 'move_to_goal':
            print("Running move to goal")
            update_motor_speed(input_omega=[robot_speed, robot_speed])
            if (calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])<0.17): # at the goal
                state = 'end'
            elif (front_ir_values[0] + front_ir_values[1]) / 2 > 800: # object detected, wall-follow
                prev = state
                state = 'wall_following'
                hit_point.append([gps_values[0], gps_values[1]])
            
        # follows perimeter of obstacle.
        elif state == 'wall_following':
            print("Running wall following")
            left_wall = (left_ir_values[0]) > 80
            front_wall = ((front_ir_values[0] + front_ir_values[1]) / 2) > 80
            right_wall = (right_ir_values[1]) > 80
            print("Left wall: ", left_ir_values[0])
            print("Front wall: ", front_ir_values[0], " and ", front_ir_values[1])
            print("Right wall: ", right_ir_values[1])

            angle_to_goal = calculate_target_angle(gps_values, goal_pos)
            open_path = is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall)
            prev_distance = calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_pos[0], goal_pos[1])
            curr_distance = calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])
            # print("Previous distance: ", prev_distance)
            # print("Current Distance: ", curr_distance)

            if front_wall: # obstacle in front
                print("Running front wall")
                print("Turn Direction: ", turn_direction)
                if turn_direction == 'CW':
                    update_motor_speed(input_omega=[-1*robot_speed, robot_speed])
                elif turn_direction == 'CCW':
                    update_motor_speed(input_omega=[robot_speed, -1*robot_speed])
                wf_prev = wf_state
                wf_state = 'front'
                
            elif (on_m_line or on_temp_line) and is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall) and prev == 'wall_following': # determine to leave wall-following
                print("Leaving wall")
                if curr_distance < prev_distance:
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'
                    wf_state = ""
                    ewf_prev = ""
                    turn_direction = 'CW'
        

            elif right_wall ^ left_wall: # moves forward while wall detected
                if open_path and (curr_distance < prev_distance):
                    print("Direction to goal is open")
                    leave_point.append([gps_values[0], gps_values[1]])
                    temp_m_line = calculate_slope(goal_pos[0], goal_pos[1], gps_values[0], gps_values[1])
                    prev = state
                    state = 'align_robot_heading'
                    wf_prev = ""
                    wf_state = ""
                    turn_direction = 'CCW'
                print("Running Right/Left Wall") 
                update_motor_speed(input_omega=[robot_speed, robot_speed])
                wf_prev = wf_state
                wf_state = 'right_wall'

            else: # turn around corners
                print("Running else")
                if turn_direction == 'CW':
                    update_motor_speed(input_omega=[robot_speed, robot_speed/corner_turn_speed])
                elif turn_direction == 'CCW':
                    update_motor_speed(input_omega=[robot_speed/corner_turn_speed, robot_speed]) 
                prev = state

        elif state == 'end': # end state
            print("Running end state")
            update_motor_speed(input_omega=[0, 0, 0]) # stop
            endtime = robot.getTime()
            elapsedtime = endtime-starttime
            print(f"Time taken to reach goal: {elapsedtime:.2f} seconds")
            break
      
        elif(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])< 0.1): # fallback end state
            state = 'end'
            break

        
    pass 