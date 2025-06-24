from controller import GPS
from controller import DistanceSensor
from controller import PositionSensor
from controller import LidarPoint
from controller import Display
from controller import Node
from controller import Robot
from controller import Supervisor
import math
import numpy as np

WHEEL_RADIUS = 0.029 
CHASSIS_AXLE_LENGTH = 0.22 

wheel_cirum = 2 * math.pi * WHEEL_RADIUS
encoder_unit = wheel_cirum / (2*math.pi)

goal_block = None
goal_pos = None
start_pos = None
gps = None
compass = None
motor_1 = None
motor_2 = None
pos_1 = None
pos_2 = None
ir_1 = None
ir_2 = None
ir_3 = None
ir_4 = None
ir_5 = None
ir_6 = None
ir_7 = None
ir_8 = None
trail_group_children_field = None 
gps_values = None
compass_val = None
position_value = None
ir_value = None
imu_yaw = None
robot_position = np.array([0.25, 0.0, 0.0])
robot_omega = np.array([0.0, 0.0])

# all comments have been added for readability and debugging

def init_robot(time_step=32):
    global robot, goal_block, goal_pos, start_pos, gps, compass, imu
    global motor_1, motor_2, pos_1, pos_2
    global ir_1, ir_2, ir_3, ir_4, ir_5, ir_6, ir_7, ir_8
    global trail_group_children_field 

    # create robot and time step of current world
    robot = Supervisor()
    TIME_STEP = 32

    # get node for the goal node then get positons
    goal_block = robot.getFromDef('goal')
    goal_field = goal_block.getField('translation')
    start_field = robot.getFromDef('bug').getField('translation')
    goal_pos = goal_field.getSFVec3f()
    start_pos = start_field.getSFVec3f()

    # define robot motors, set position and set velocity
    motor_1 = robot.getDevice("left_wheel_motor"); motor_2 = robot.getDevice("right_wheel_motor")
    motor_1.setPosition(float('inf')); motor_2.setPosition(float('inf'))
    motor_1.setVelocity(0.0); motor_2.setVelocity(0.0)

    # define position and infrared sensors, gps, compass, and IMU and enable
    pos_1 = robot.getDevice("left_wheel_sensor"); pos_2 = robot.getDevice("right_wheel_sensor")
    pos_1.enable(time_step); pos_2.enable(time_step)
    ir_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
    ir_sensors = [robot.getDevice(name) for name in ir_names]
    for sensor in ir_sensors:
        sensor.enable(time_step)
    (ir_1, ir_2, ir_3, ir_4, ir_5, ir_6, ir_7, ir_8) = ir_sensors
    gps = robot.getDevice("gps"); gps.enable(time_step)
    compass = robot.getDevice("compass"); compass.enable(time_step)
    imu = robot.getDevice("inertial_unit"); imu.enable(time_step)

    # Get a handle to the TRAIL_GROUP node's children field
    trail_group_node = robot.getFromDef("TRAIL_GROUP")
    if trail_group_node:
        trail_group_children_field = trail_group_node.getField("children")
    else:
        print("[Bug Robot]: ERROR - Could not find TRAIL_GROUP node.")

    print("Initializing...")
    # Return all necessary objects to the main controller
    return robot, goal_pos, start_pos, trail_group_children_field


def read_sensors_values():
    global gps_values, compass_val, position_value, ir_value, imu_yaw

    gps_values = gps.getValues()
    # print("GPS Read: ", gps_values[0], gps_values[1], gps_values[2])

    compass_val = compass.getValues()
    # print("Compass Read: ", compass_val)

    position_value = encoder_unit*np.array([pos_1.getValue(),pos_2.getValue()])
    # print("Position Read: ", position_value)

    ir_value = np.array([ir_1.getValue(),ir_2.getValue(), ir_3.getValue(), ir_4.getValue(), ir_5.getValue(), ir_6.getValue(), ir_7.getValue(), ir_8.getValue()])
    # print("IR Read: ", ir_value)

    imu_rpy = imu.getRollPitchYaw()
    imu_yaw = math.degrees(imu_rpy[2])
    if(imu_yaw < 0): imu_yaw += 360
    elif(imu_yaw > 360): imu_yaw -= 360
    # print("Orientation: ", imu_yaw)

    return gps_values,compass_val,position_value,ir_value, imu_yaw

def init_robot_state(in_pos=robot_position,in_omega=robot_omega):
    global robot_position, robot_omega
    robot_position = in_pos
    robot_omega    = in_omega

def update_robot_state():
    global robot_velocity, robot_position, robot_omega

    # updating the current theta amd position
    robot_position[2] = math.atan2(compass_val[0], compass_val[1])
    robot_position[0] = gps_values[0]
    robot_position[1] = gps_values[1]
    # print("Robot Position Compass: ", robot_position[2], Robot Position [1]: ", robot_position[1], Robot Position [0]: ", robot_position[0])

def update_motor_speed(input_omega=robot_omega):
    motor_1.setVelocity(input_omega[0])
    motor_2.setVelocity(input_omega[1])
    # print("Motor 1 Speed Updated to: ", input_omega[0], "Motor 2 Speed Updated to: ", input_omega[1])


# the goal will be calculated based on 0 degrees started at north going clockwise
# the robot will be calculated based on 0 degrees started at east going counter clockwise
# this function calculates the goal angle and then normalizes it on a 0-360 degrees
# then it will negate the angle so it will be based on the counter clockwise
# then add 90 to start 0 degrees from east, and add 360 to get an angle betweeen 0-360
def calculate_target_angle(robot_pos, goal_pos):
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

# boolean function: rotates robot if not pointed the goal point
# uses the target angle (angle needed to face goal) and yaw
# angle (direction robot is currently facing) to determine
def align_to_M(target_angle, yaw_angle, threshold = 0.5):
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
    #    print("Both angles less than 180")
        if target_angle < yaw_angle:
            turn = 'right'
    #        print("Target angle is less than yaw. Turn right")
        else: 
            turn = 'left'
    #        print("Target angle is more than yaw. Turn left")
    elif (target_angle <= 360 and yaw_angle <= 360) and (target_angle >=180 and yaw_angle >= 180):
    #    print("Both angles greater than 180 and less than 360")
        if target_angle < yaw_angle:
            turn = 'right'
    #        print("Target angle is less than yaw. Turn right")
        else: 
            turn = 'left'
    #        print("Target angle is more than yaw. Turn left")
    else:
        if yaw_angle > halfway:
            turn = 'left'
    #        print("Yaw is greater than halfway. Turn left")
        else: 
            turn = 'right'
    #        print("Yaw is less than halfway. Turn right")

    if abs(difference) < threshold:
        print("Aligned")
        return True
    else:
        if turn == 'left':
    #        print("Turning left")
            update_motor_speed(input_omega=[-ts, ts/20])
        else: 
    #        print("Turning right")
            update_motor_speed(input_omega=[ts/20, -ts])
        return False

def calculate_euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def calculate_slope(x1, y1, x2, y2):
    if (x1-x2) == 0:
        return x1
    return((y1-y2)/(x1-x2))

def is_on_M_line(currX, currY, goalX, goalY, m_line, threshold=0.1):
    slope = calculate_slope(currX, currY, goalX, goalY)
    return abs(m_line - slope) < threshold

# defines the m_line
def m_line_b(x, y, m):
    return -m*x + y

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
    #print("Front angle: ", front_angle, " Right angle: ", right_angle, " Left angle: ", left_angle)

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

def normalize_angle(angle):
    if angle < 0:
        angle += 360
    if angle > 360:
        angle -= 360
    return angle
