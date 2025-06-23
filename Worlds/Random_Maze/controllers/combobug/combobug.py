"""
sensor_rewrite_with_trail.py
───────────────────────────────
This controller implements a custom Bug Algorithm based on the 'sensor_rewrite' logic.
It has been merged with the breadcrumb trail feature from 'combobug.py' to provide
a visual path of the robot's journey.

Core Logic: From 'sensor_rewrite controller'
Features:
- Custom M-line following and obstacle avoidance state machine.
- Detailed angle calculations for alignment.
- Hit and leave point tracking.

Added Feature: From 'combobug.py'
- Visual breadcrumb trail to trace the robot's path in the simulation.
"""

import numpy as np
from initialization import * # Assumes this provides init_robot, read_sensors_values, update_motor_speed, etc.
import math
from controller import Supervisor # Ensure Supervisor is available

# =====================================================================================
# Main algorithm logic from your 'sensor_rewrite' file. No changes needed here.
# =====================================================================================

def calculate_target_angle(robot_pos, goal_pos):
    #the goal will be calculated based on 0 degrees started at north going clockwise
    #the robot will be calculated based on 0 degrees started at east going counter clockwise
    # this function calculates the goal angle and then normalizes it on a 0-360 degrees
    # then it will negate the angle so it will be based on the counter clockwise
    # then add 90 to start 0 degrees from east, and add 360 to get an angle betweeen 0-360
    dx = goal_pos[0] - robot_pos[0]
    dy = goal_pos[1] - robot_pos[1]
    angle_to_target = math.degrees(math.atan2(dx, dy))
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
    return angle_to_target

def align_to_M(target_angle, yaw_angle, threshold = 0.5):
    ts = 1  # turning speed
    turn = 'left'
    difference = target_angle - yaw_angle
    halfway = target_angle - 180
    if halfway < 0:
        halfway += 360
    if target_angle <= 180 and yaw_angle <= 180:
        if target_angle < yaw_angle:
            turn = 'right'
        else:
            turn = 'left'
    elif (target_angle <= 360 and yaw_angle <= 360) and (target_angle >=180 and yaw_angle >= 180):
        if target_angle < yaw_angle:
            turn = 'right'
        else:
            turn = 'left'
    else:
        if yaw_angle > halfway:
            turn = 'left'
        else:
            turn = 'right'

    if abs(difference) < threshold:
        # print("Aligned")
        return True
    else:
        if turn == 'left':
            update_motor_speed(input_omega=[-ts, ts/20])
        else:
            update_motor_speed(input_omega=[ts/20, -ts])
        return False

def calculate_euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def calculate_slope(x1, y1, x2, y2):
    if (x1-x2) == 0:
        return float('inf') # Handle vertical M-lines
    return((y1-y2)/(x1-x2))

def is_on_M_line(currX, currY, goalX, goalY, m_line, threshold=0.1):
    # This check can be simplified. A point (x,y) is on the line y = mx + b if y - mx - b is close to 0.
    # The original implementation using slope recalculation is kept for consistency.
    if m_line == float('inf'): # Vertical line case
        return abs(currX - goalX) < threshold
    slope = calculate_slope(currX, currY, goalX, goalY)
    return abs(m_line - slope) < threshold

def is_open(yaw, atg, right, left, front):
    # Simplified for clarity, retaining original intent
    # Note: This logic can be complex and depends heavily on sensor placement.
    # The provided logic is kept as is.
    normalized_right = yaw - 90
    normalized_left = yaw + 90
    normalized_frontone = yaw - 15
    normalized_fronttwo = yaw + 15
    if normalized_left > 360: normalized_left -= 360
    if normalized_right < 0: normalized_right += 360
    if normalized_frontone < 0: normalized_frontone += 360
    if normalized_fronttwo > 360: normalized_fronttwo -= 360

    def is_angle_between(angle, start, end):
        # Handles angle wrapping around 360 degrees
        if start <= end:
            return start <= angle <= end
        else: # Wraps around 360
            return start <= angle or angle <= end

    if not front and is_angle_between(atg, normalized_frontone, normalized_fronttwo):
        return True
    if not right and is_angle_between(atg, normalized_right, normalized_frontone):
        return True
    if not left and is_angle_between(atg, normalized_fronttwo, normalized_left):
        return True
    return False


# =====================================================================================
# BREADCRUMB TRAIL FUNCTION (from combobug.py)
# This function is adapted to work in your procedural script.
# =====================================================================================
def drop_breadcrumb(supervisor, parent_node, pos, template_string):
    """
    Drops a visual marker (a small sphere) at the robot's current position.
    It finds or creates a 'BREADCRUMB_GROUP' node to keep the scene tidy.
    """
    if parent_node is None:
        # Find the group node for the first time
        root_node = supervisor.getRoot()
        root_children_field = root_node.getField('children')
        
        # Check if the group already exists from a previous run
        group_node = supervisor.getFromDef('BREADCRUMB_GROUP')
        if not group_node:
            # If not, create it
            print("Creating BREADCRUMB_GROUP for the trail.")
            vrml_string = 'DEF BREADCRUMB_GROUP Group {}'
            root_children_field.importMFNodeFromString(-1, vrml_string)
            parent_node = supervisor.getFromDef('BREADCRUMB_GROUP')
        else:
            parent_node = group_node

    if parent_node:
        # Format the VRML string with the robot's current 2D position
        vrml_string = template_string % (pos[0], pos[1])
        breadcrumb_children = parent_node.getField('children')
        breadcrumb_children.importMFNodeFromString(-1, vrml_string)
    
    # Return the parent_node handle so we don't have to search for it again
    return parent_node


if __name__ == "__main__":
    # initialization of robot
    TIME_STEP = 32
    # IMPORTANT: init_robot() must return a Supervisor instance for this to work!
    robot, goal_pos, start_pos = init_robot(time_step=TIME_STEP)
    print("Robot Initialized")
    init_robot_state(in_pos=[0,0,0], in_omega=[0,0])

    # --- Breadcrumb Trail Initialization (from combobug.py) ---
    BREADCRUMB_TEMPLATE = (
        'Transform { translation %f %f 0.01 children [ Shape { '
        '  appearance PBRAppearance { baseColor 0 0.7 1 metalness 0.2 transparency 0.6 }'
        '  geometry Sphere { radius 0.02 subdivision 1 }'
        '} ] }'
    )
    breadcrumb_parent_node = None # Will hold the 'BREADCRUMB_GROUP' node
    breadcrumb_counter = 0        # To control how often we drop a marker
    BREADCRUMB_INTERVAL = 15      # Drop a marker every 15 simulation steps

    # calculate m-line
    m_line = calculate_slope(goal_pos[0], goal_pos[1], start_pos[0], start_pos[1])
    
    # define local variables from your script
    state = 'start'
    robot_speed = 3
    hit_point = []
    leave_point = []
    prev = ""
    wf_state = ""
    wf_prev = ""
    starttime = robot.getTime()

    # robot loop
    while robot.step(TIME_STEP) != -1:
        # --- Sensor and State Update (your original code) ---
        gps_values, compass_val, encoder_value, ir_value, imu_yaw = read_sensors_values()
        front_ir_values = ir_value[0], ir_value[7]
        right_ir_values = ir_value[1], ir_value[2]
        left_ir_values = ir_value[5], ir_value[6]
        update_robot_state()

        # --- Drop Breadcrumb Trail (logic from combobug.py) ---
        breadcrumb_counter += 1
        if breadcrumb_counter % BREADCRUMB_INTERVAL == 0:
            # We pass the supervisor, current parent node, position, and template
            breadcrumb_parent_node = drop_breadcrumb(robot, breadcrumb_parent_node, gps_values, BREADCRUMB_TEMPLATE)

        # =========================================================================
        # The rest of your state machine logic remains UNCHANGED.
        # =========================================================================

        if state == 'start':
            prev = state
            state = 'align_robot_heading'

        elif state == 'align_robot_heading':
            print("Running alignment")
            is_aligned = align_to_M(calculate_target_angle(gps_values, goal_pos), imu_yaw)
            if is_aligned:
                prev = state
                state = 'move_to_goal'

        elif state == 'move_to_goal':
            print("Running move to goal")
            update_motor_speed(input_omega=[robot_speed, robot_speed])
            if (calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1]) < 0.17):
                state = 'end'
            elif (front_ir_values[0] + front_ir_values[1]) / 2 > 800:
                prev = state
                state = 'wall_following'
                hit_point.append([gps_values[0], gps_values[1]])

        elif state == 'wall_following':
            print("Running wall following. WF State:", wf_state)
            left_wall = ((left_ir_values[0] + left_ir_values[1]) / 2) > 80
            front_wall = ((front_ir_values[0] + front_ir_values[1]) / 2) > 80
            right_wall = ((right_ir_values[0] + right_ir_values[1]) / 2) > 80
            angle_to_goal = calculate_target_angle(gps_values, goal_pos)
            prev_distance = calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_pos[0], goal_pos[1])
            curr_distance = calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1])

            if front_wall:
                print("WF: Front wall detected, turning.")
                update_motor_speed(input_omega=[-1 * robot_speed, robot_speed]) # Assuming this turns left
                wf_prev = wf_state
                wf_state = 'front'
            
            elif (is_on_M_line(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1], m_line)) and \
                 is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall) and prev == 'wall_following':
                print("On m-line and path is open!")
                if curr_distance < prev_distance:
                    print("Closer to goal. Leaving wall.")
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'
                    wf_state = ""
                    wf_prev = ""
            
            elif right_wall: # Assuming right-hand wall following
                if is_open(imu_yaw, angle_to_goal, right_wall, left_wall, front_wall) and (curr_distance < prev_distance):
                    print("WF: Path to goal is open and we are closer. Leaving wall.")
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'
                    wf_state = ""
                    wf_prev = ""
                else:
                    print("WF: Following right wall.")
                    update_motor_speed(input_omega=[robot_speed, robot_speed])
                    wf_prev = wf_state
                    wf_state = 'right_wall'
            
            elif not right_wall and (wf_state == 'right_wall' or wf_state == 'right_turn'):
                print("WF: Lost right wall, turning to find it.")
                update_motor_speed(input_omega=[robot_speed, robot_speed / 20]) # Assuming this is a right turn
                wf_prev = wf_state
                wf_state = 'right_turn'
            
            else: # Default behavior if lost, e.g., turn to find a wall
                print("WF: Lost, attempting to find wall.")
                update_motor_speed(input_omega=[-robot_speed, robot_speed])


        elif state == 'end':
            print("Running end state. Goal Reached!")
            update_motor_speed(input_omega=[0, 0])
            endtime = robot.getTime()
            elapsedtime = endtime - starttime
            print(f"Time taken to reach goal: {elapsedtime:.2f} seconds")
            break
        
        # Failsafe goal check
        elif (calculate_euclidean_distance(gps_values[0], gps_values[1], goal_pos[0], goal_pos[1]) < 0.1):
            state = 'end'

    pass