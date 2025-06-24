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
    m_line = calculate_slope(goal_pos[0], goal_pos[1], start_pos[0], start_pos[1])
    original_m_line = m_line
    b = m_line_b(start_pos[0], start_pos[1], m_line)
    state = 'start'
    robot_speed = 3
    corner_turn_speed = 12
    hit_point = []
    leave_point = []
    turn_direction = 'left'
    starttime = robot.getTime()
    trail_counter = 0

    while robot.step(TIME_STEP) != -1:
        
        gps_values, compass_val, encoder_value, ir_value, imu_yaw = read_sensors_values()
        
        # logic to drop trail sphere
        trail_counter += 1
        if trail_counter % 15 == 0 and trail_group_children_field is not None:
            pos = gps_values
            
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
            trail_group_children_field.importMFNodeFromString(-1, sphere_string)

        front_ir_values = ir_value[0], ir_value[7]
        right_ir_values = ir_value[1], ir_value[2]
        left_ir_values = ir_value[5], ir_value[6]
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
            if (calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])<0.17):
                state = 'end'
            elif (front_ir_values[0] + front_ir_values[1]) / 2 > 800:
                prev = state
                state = 'wall_following'
                hit_point.append([gps_values[0], gps_values[1]])

        # follows perimeter of obstacle.
        elif state == 'wall_following':
            print("Running wall following")
            left_wall = ((left_ir_values[0] + left_ir_values[1]) /2) > 80
            front_wall = ((front_ir_values[0] + front_ir_values[1]) / 2) > 80
            right_wall = ((right_ir_values[0] + right_ir_values[1]) /2) > 80
            angle_to_goal = calculate_target_angle(gps_values, goal_pos)
            prev_distance = calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_pos[0], goal_pos[1])
            curr_distance = calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])
            # print("Previous distance: ", prev_distance)
            # print("Current Distance: ", curr_distance)
            # print("WF State: ", wf_state)
            # print("WF Prev: ", wf_prev)

            if front_wall: #obstacle in front
                print("Running front wall")
                update_motor_speed(input_omega=[-1*robot_speed, robot_speed])
                wf_prev = wf_state
                wf_state = 'front'
            
            elif (is_on_M_line(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1], m_line)) and is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall) and prev == 'wall_following': # determine to leave wall following
                print("On m-line!")
                if curr_distance < prev_distance:
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'
                    wf_state = ""
                    ewf_prev = ""
            
            elif right_wall: # moves forward while wall detected
                if is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall) and (curr_distance < prev_distance):
                    print("Direction to goal is open")
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'
                    wf_prev = ""
                    wf_state = ""
                print("Running Right Wall") 
                update_motor_speed(input_omega=[robot_speed, robot_speed])
                wf_prev = wf_state
                wf_state = 'right_wall'
            
            elif (wf_state == 'right_wall' and right_wall==False) or (wf_state == 'right_turn' and right_wall==False): # turn around corners
                print("Running right turn")
                update_motor_speed(input_omega=[robot_speed, robot_speed/corner_turn_speed])
                prev = state
                wf_prev = wf_state
                wf_state = 'right_turn'
        
        elif state == 'end': # end state
            print("Running end state")
            update_motor_speed(input_omega=[0, 0, 0]) # stop
            endtime = robot.getTime()
            elapsedtime = endtime - starttime
            print(f"Time taken to reach goal: {elapsedtime:.2f} seconds")
            break
        
        elif(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])< 0.1): # fallback end state
            state = 'end'
            break
    pass