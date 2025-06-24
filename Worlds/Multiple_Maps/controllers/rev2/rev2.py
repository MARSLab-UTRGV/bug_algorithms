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
    prev = ''
    m_line = calculate_slope(goal_pos[0], goal_pos[1], start_pos[0], start_pos[1])
    state = 'start'
    robot_speed = 3
    corner_turn_speed = 12
    hit_point = []      # x, y
    leave_point = []    # x, y
    starttime = robot.getTime()
    wf_state = ''
    wf_prev = ''
    trail_counter = 0
    next_turn = 'left'

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
            prev = state
            state = 'align_robot_heading'

        # checks to see if robot is aligned. if align, start moving
        if state == 'align_robot_heading':
            print("Running align robot heading state")
            is_aligned = align_to_M(calculate_target_angle(gps_values, goal_pos), imu_yaw)#need to set the goal here for the object to orient itself to 
            # print("target angle: ", calculate_target_angle(gps_values, goal_pos))
            # print("imu yaw: ", imu_yaw)
            if is_aligned: 
                prev = state
                state = 'move_to_goal'

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
                    
            
        # follows perimeter of obstacle.
        elif state == 'wall_following':
            print("Running wall_following state")
            left_wall = left_ir_values[0] > 80
            front_wall = (front_ir_values[0] + front_ir_values[1])/2 > 80
            right_wall = right_ir_values[0] > 80
            # print("Left wall: ", left_ir_values[0])
            # print("Front wall: ", front_ir_values[0], " and ", front_ir_values[1])
            # print("Right wall: ", right_ir_values[1])

            angle_to_goal = calculate_target_angle(gps_values, goal_pos)
            open_path = is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall)
            prev_distance = calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_pos[0], goal_pos[1])
            curr_distance = calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])
            # print("Previous distance: ", prev_distance)
            # print("Current Distance: ", curr_distance)

            if front_wall: # obstacle in front
                print("Running front wall")
                wf_prev = wf_state
                wf_state = 'front_wall'
                if next_turn == 'left':
                    update_motor_speed(input_omega=[-1*robot_speed, robot_speed])
                else:
                    update_motor_speed(input_omega=[robot_speed, -1*robot_speed])

            elif right_wall ^ left_wall: # moves forward while wall detected
                print("side wall detected")
                wf_prev = wf_state
                wf_state = 'wall detected'
                update_motor_speed(input_omega=[robot_speed, robot_speed])
                # if right_wall: print("touching wall on right")
                # elif left_wall: print("touching wall on left")
                if (open_path and curr_distance < prev_distance) and not wf_prev == 'else': # determine to leave wall-following
                    print("Direction to goal is open")
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'
                    wf_state = ''
                    wf_prev = ''
                    if next_turn == 'left':
                        next_turn = 'right'
                    else:
                        next_turn = 'left'
                    # print("Next turn changed to: ", next_turn)

            else: # turn around corners
                print("Running else")
                wf_prev = wf_state
                wf_state = 'else'
                if next_turn == 'left':
                    update_motor_speed(input_omega=[robot_speed, robot_speed/corner_turn_speed])
                elif next_turn == 'right':
                    update_motor_speed(input_omega=[robot_speed/corner_turn_speed, robot_speed])               
                prev = state

        elif state == 'end': #end state
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
            
                    