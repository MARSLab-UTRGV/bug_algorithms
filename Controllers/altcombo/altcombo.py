"""sensor_rewrite controller."""

import numpy as np
from initialization import *
import math

#Here is a description of the new algorithm that we want to work 
#This algorithm is an extension of the ComboBug algorithm that I created to merge the concepts of
#Common-sense bug algorithms and m-line bug algorithms

# returns distance between two given points
def calculate_euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

# returns slope of the m-line between goal and start
def calculate_slope(x1, y1, x2, y2):
    if (x1-x2) == 0:
        return x1
    return((y1-y2)/(x1-x2))

# boolean function: Checks if robot is on m_line
# write in the m-line to compare instead of calculating the slope every time
def is_on_M_line(currX, currY, goalX, goalY, m_line, threshold=0.1):
    slope = calculate_slope(currX, currY, goalX, goalY)
    return abs(m_line - slope) < threshold

def m_line_b(x, y, m):
    return -m*x + y

def equivalent_point_on_mline(x, b, m):
    y = m*x + b
    return y, x

def is_open(yaw, atg, right, left, front):
    print("Checking if open")
    print("Right Wall: ", right)
    print("Left Wall: ", left)
    print("Front Wall: ", front)
    print("yaw = ", yaw)
    print("atg = ", atg)
    normalized_right = yaw - 90
    normalized_left = yaw + 90
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

    #print("front_angle: ", front_angle)
    #print("Within front: ", (atg>=front_angle[0] and atg <= front_angle[1]))
    #print("right_angle: ", right_angle)
    #print("Within right: ", (atg >= right_angle[0] and atg < right_angle[1]))
    #print("left_angle: ", left_angle)
    #print("Within left: ", (atg > left_angle[0] and atg <= left_angle[1]))
    #print("Front overlap: ", (front_angle[0] > front_angle[1]))
    #print("Front overlap open: ", (atg >= (front_angle[0]-360) and atg <= front_angle[1]))
    #print("Both front: ", (front_angle[0] > front_angle[1]) and (atg >= (front_angle[0]-360) and atg <= front_angle[1]))
    #print("Right overlap: ", (right_angle[0] > right_angle[1]))
    #print("Right overlap open: ", (atg >= (right_angle[0]-360) and atg <= right_angle[1]))
    #print("Both right: ",(right_angle[0] > right_angle[1]) and (atg >= (right_angle[0]-360) and atg <= right_angle[1]) )
    #print("Left overlap: ", (left_angle[0] > left_angle[1]))
    #print("Left overlap open: ", (atg >= (left_angle[0]) and atg <= left_angle[1]+360))
    #print("Both left: ", (left_angle[0] > left_angle[1]) and (atg >= (left_angle[0]) and atg <= left_angle[1]+360))

    if front == False:
        #print("in front")
        if (atg >= front_angle[0] and atg <= front_angle[1]):
            #print("Normal angle open in Front")
            return True      
        if (front_angle[0] > front_angle[1]) and (atg >= (front_angle[0]-360) and atg <= front_angle[1]):
            #print("Angle in overlap open in front")
            return True
    if right == False:
        #print("in right")
        if (atg >= right_angle[0] and atg <= right_angle[1]):
            # print("Right open")
            return True
        if (right_angle[0] > right_angle[1]) and (atg >= (right_angle[0]-360) and atg <= right_angle[1]):
            #print("Angle in overlap within right")
            return True
    if left == False:
        print("in left")
        if (atg >= left_angle[0] and atg < left_angle[1]):
            # print("Left Open")
            return True
        if (left_angle[0] > left_angle[1]) and (atg >= (left_angle[0]) and atg <= left_angle[1]+360):
            print("Angle in overlap within left")
            return True
        
    # print("Not Open")
    return False


if __name__ == "__main__":
    # initialization of robot
    TIME_STEP = 32
    robot, goal_pos, start_pos = init_robot(time_step=TIME_STEP)
    print("Robot Initialized")
    init_robot_state(in_pos=[0,0,0], in_omega=[0,0])
    prev = ""
    wf_state = ""
    wf_prev = ""

    # calculate m-line
    m_line = calculate_slope(goal_pos[0], goal_pos[1], start_pos[0], start_pos[1])
    original_m_line = m_line
    temp_m_line = None
    # calculate y-intercept
    b = m_line_b(start_pos[0], start_pos[1], m_line)

    # define local variables
    state = 'start'
    robot_speed = 3
    hit_point = []      # x, y
    leave_point = []    # x, y
    turn_direction = 'CW'

    starttime = robot.getTime()


    # robot loop
    while robot.step(TIME_STEP) != -1:

        gps_values, compass_val, encoder_value, ir_value, imu_yaw = read_sensors_values()

        front_ir_values = ir_value[0], ir_value[7]
        right_ir_values = ir_value[1], ir_value[2]
        left_ir_values = ir_value[5], ir_value[6]
        on_m_line = is_on_M_line(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1], m_line)
        on_temp_line = False
        if temp_m_line != None:
            on_temp_line = is_on_M_line(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1], temp_m_line)

        update_robot_state()

        # start state will move it to align robot heading. this step can be commented out if necessary.
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

        # moves the robot along the m-line
        elif state == 'move_to_goal':
            print("Running move to goal")
            update_motor_speed(input_omega=[robot_speed, robot_speed])
            # if at the goal, move to end state
            if (calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])<0.17):
                state = 'end'
            # if a wall is detected in front of the robot, begin wall following
            elif (front_ir_values[0] + front_ir_values[1]) / 2 > 800:
                # divide into two: a comparison of where it is relative to the original m-line and current m-line
                ##############################
                # if x,y on original m-line is to the left of the robot, turning direction becomes left, else turning direction becomes right
                prev = state
                state = 'wall_following'
                hit_point.append([gps_values[0], gps_values[1]])
            
        # follows perimeter of obstacle.
        elif state == 'wall_following':
            print("Running wall following")
            #calculate the if a wall is on the left, right or front of the robot
            left_wall = ((left_ir_values[0] + left_ir_values[1]) /2) > 80
            front_wall = ((front_ir_values[0] + front_ir_values[1]) / 2) > 80
            right_wall = ((right_ir_values[0] + right_ir_values[1]) /2) > 80
            #print("Left wall: ", left_wall)
            #print("Front wall: ", front_wall)
            #print("Right wall: ", right_wall)

            #calculate angle to goal
            angle_to_goal = calculate_target_angle(gps_values, goal_pos)

            #calculate if path is open
            open_path = is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall)


            #calculates the distance from the previous hit point to the goal 
            #calculates the distance from the current position to the goal
            prev_distance = calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_pos[0], goal_pos[1])
            curr_distance = calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])
            #print("Previous distance: ", prev_distance)
            #print("Current Distance: ", curr_distance)

            #print("WF State: ", wf_state)
            #print("WF Prev: ", wf_prev)
            

            # found wall in front, determine turn based on the turn direction
            if front_wall: 
                print("Running front wall")
                print("Turn Direction: ", turn_direction)
                if turn_direction == 'CW':
                    update_motor_speed(input_omega=[-1*robot_speed, robot_speed])
                elif turn_direction == 'CCW':
                    update_motor_speed(input_omega=[robot_speed, -1*robot_speed])
                wf_prev = wf_state
                wf_state = 'front'
                

            # is on current m-line after wall following, creates leave point and moves into alignment state 
            # To correctly determine leave points: check that the obstacle is not between the robot and goal using left and right wall sensors
            elif (on_m_line or on_temp_line) and is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall) and prev == 'wall_following':
                print("On m-line!")
                # elif determines if the robot is on the m-line and has made turns
                # if statement determines that the robot is not above/further away from the goal than its original hit point
                ########The if statement below may need to be rearranged to a different spot... for the purpose of checking when the way to T is free again
                if curr_distance < prev_distance:
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'
                    wf_state = ""
                    ewf_prev = ""
        

            # following wall on the right-side of the robot, moves forward and straight
            elif right_wall ^ left_wall:
                if open_path and (curr_distance < prev_distance):
                    print("Direction to goal is open")
                    leave_point.append([gps_values[0], gps_values[1]])
                    temp_m_line = calculate_slope(goal_pos[0], goal_pos[1], gps_values[0], gps_values[1])
                    prev = state
                    state = 'align_robot_heading'
                    wf_prev = ""
                    wf_state = ""
                    if turn_direction == 'CW':
                        turn_direction = 'CCW'
                    else:
                        turn_direction = 'CW'
                print("Running Right/Left Wall") 
                update_motor_speed(input_omega=[robot_speed, robot_speed])
                wf_prev = wf_state
                wf_state = 'right_wall'

            # if too far from the wall and left following, turn back towards the wall (right-turn)
            # if too far from the wall and right following, turn back towards the wall (left-turn)    
            #elif (wf_state == 'right_wall' and right_wall==False) or (wf_state == 'right_turn' and right_wall==False):
            #    #check orientation to the goal for the current m-line and then see if there is an obstacle in that direction. set new m-line and follow
            #    print("Running right turn")
            #    #check where the object is compared to the goal and in turn check the sensors responsible. 
            #    update_motor_speed(input_omega=[robot_speed, robot_speed/20])
            #    # print("Prev Changed 2")
            #    prev = state
            #    wf_prev = wf_state
            #    wf_state = 'right_turn'
        
             # if too far from the wall
            else:
                print("Running else")
                if turn_direction == 'CW':
                    update_motor_speed(input_omega=[robot_speed, robot_speed/10])
                elif turn_direction == 'CCW':
                    update_motor_speed(input_omega=[robot_speed/10, robot_speed])                # print("Prev Changed 2")
                prev = state


        # end state to stop
        elif state == 'end':
            print("Running end state")
            update_motor_speed(input_omega=[0, 0, 0]) #end
            endtime = robot.getTime()
            elapsedtime = endtime-starttime
            print(f"Time taken to reach goal: {elapsedtime:.2f} seconds")
            break

        # determines end state if all other if statements are false and condition holds true        
        elif(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])< 0.1):
            state = 'end'
            break

        
    pass 