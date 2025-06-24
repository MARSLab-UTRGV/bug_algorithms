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
        print("Aligned")
        return True
    else:
        if turn == 'left':
            print("Turning left")
            update_motor_speed(input_omega=[-ts, ts/20])
        else: 
            print("Turning right")
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
    # print("Slope: ", (y1-y2)/(x1-x2))
    return((y1-y2)/(x1-x2))


def is_open(yaw, atg, right, left, front):
    print("Checking if open")
    #print("Right Wall: ", right)
    #print("Left Wall: ", left)
    #print("Front Wall: ", front)
    #print("yaw = ", yaw)
    #print("atg = ", atg)
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


    # calculate m-line
    m_line = calculate_slope(goal_pos[0], goal_pos[1], start_pos[0], start_pos[1])
    # define local variables
    state = 'align_robot_heading'
    wf_state = ""
    wf_prev = ""
    robot_speed = 3
    forward_left_speeds = [robot_speed, robot_speed]
    far_from_wall_counter = 0
    close_to_wall_counter = 0
    hit_point = []      # x, y
    leave_point = []    # x, y

    starttime = robot.getTime()

    # Determine the default turn style of turning to the left and right_wall following
    next_turn = 'left'

    # robot loop
    while robot.step(TIME_STEP) != -1:

        # trash variable for unused sonar. can be taken out of initialization.py
        gps_values, compass_val, encoder_value, ir_value, imu_yaw = read_sensors_values()
        # print("Sensor Read Complete")
        # print("GPS Values: ", gps_values[0], gps_values[1])
        front_ir_values = ir_value[0], ir_value[7]
        right_ir_values = ir_value[1], ir_value[2]
        left_ir_values = ir_value[5], ir_value[6]

        left_wall = (left_ir_values[0] + left_ir_values[1])/2 > 80
        front_wall = (front_ir_values[0] + front_ir_values[1])/2 > 80
        right_wall = (right_ir_values[0] + right_ir_values[0])/2 > 80
        # print("Left Values: ", left_ir_values)
        # print("Front Values: ", front_ir_values)
        # print("Right Values: ", right_ir_values)
        # print("Left Wall: ", left_wall)
        # print("Front Wall: ", front_wall)
        # print("Right Wall: ", right_wall)

        update_robot_state()
        # print("Current: ", state)
        # print("Previous: ", prev)
            

        # checks to see if robot is aligned. if align, start moving
        if state == 'align_robot_heading':
            print("Running align robot heading state")
            is_aligned = align_to_M(calculate_target_angle(gps_values, goal_pos), imu_yaw)#need to set the goal here for the object to orient itself to 
            if is_aligned: 
                prev = state
                state = 'move_to_goal'
        #    calculate_slope(goal_pos[0], goal_pos[1], gps_values[0], gps_values[1])

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
            # left_wall = left_ir_values[0] > 80
            # front_wall = ((front_ir_values[0] + front_ir_values[1]) / 2) > 80
            # right_wall = right_ir_values[1] > 80
            #print("Left Values: ", left_ir_values)
            #print("Front Values: ", front_ir_values)
            #print("Right Values: ", right_ir_values)
            #print("Left Wall: ", left_wall)
            #print("Front Wall: ", front_wall)
            #print("Right Wall: ", right_wall)

            angle_to_goal = calculate_target_angle(gps_values, goal_pos)
            open_path = is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall)

            prev_distance = calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_pos[0], goal_pos[1])
            curr_distance = calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])

            # print("Next Turn: ", next_turn)

            # found wall in front, default turn to left
            if front_wall: 
                print("Running front wall")
                if next_turn == 'left':
                    update_motor_speed(input_omega=[-1*robot_speed, robot_speed])
                else:
                    update_motor_speed(input_omega=[robot_speed, -1*robot_speed])

            # following wall on the right or left with exclusive or (XOR)
            elif right_wall ^ left_wall:
                update_motor_speed(input_omega=[robot_speed, robot_speed])
                # print("Running Wall")
                if right_wall:
                    print("touching wall on right")
                elif left_wall:
                    print("touching wall on left")
                if open_path and curr_distance < prev_distance:
                    print("Direction to goal is open")
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'
                    if next_turn == 'left':
                        next_turn = 'right'
                    else:
                        next_turn = 'left'
                    # print("Next turn changed to: ", next_turn)

            # if too far from the wall
            else:
                print("Running else")
                if next_turn == 'left':
                    update_motor_speed(input_omega=[robot_speed, robot_speed/10])
                elif next_turn == 'right':
                    update_motor_speed(input_omega=[robot_speed/10, robot_speed])                # print("Prev Changed 2")
                prev = state

        elif state == 'end':
            print("Running end state")
            update_motor_speed(input_omega=[0, 0, 0]) #end
            endtime = robot.getTime()
            elapsedtime = endtime-starttime
            print(f"Time taken to reach goal: {elapsedtime:.2f} seconds")
            break
                
        elif(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])< 0.1):
            state = 'end'
            break
        
    pass
            
                    