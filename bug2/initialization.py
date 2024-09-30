
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
# motor_3 = None
pos_1 = None
pos_2 = None
# pos_3 = None
ir_1 = None
ir_2 = None
ir_3 = None
ir_4 = None
ir_5 = None
ir_6 = None
ir_7 = None
ir_8 = None
gps_values = None
compass_val = None
position_value = None
ir_value = None
imu_yaw = None
robot_position = np.array([0.25, 0.0, 0.0])
robot_omega = np.array([0.0, 0.0])


def init_robot(time_step=32):

    global robot
    global goal_block
    global goal_pos
    global start_pos
    global gps
    global compass
    global imu
    global motor_1
    global motor_2
    global pos_1
    global pos_2

    global ir_1
    global ir_2
    global ir_3
    global ir_4
    global ir_5
    global ir_6
    global ir_7
    global ir_8

    # create the Robot instance.
    robot = Supervisor()
    
    # get the time step of the current world.
    TIME_STEP = 32

    # get the node for the goal node
    goal_block = robot.getFromDef('goal')
    # print("goal block: ", goal_block)

    # Get the position field from the node
    goal_field = goal_block.getField('translation')
    start_field = robot.getFromDef('bug').getField('translation')

    # Get the start position and goal position
    goal_pos = goal_field.getSFVec3f()
    start_pos = start_field.getSFVec3f()

    # print("Goal pos: ", goal_pos)
    # print("Start pos: ", start_pos)

    # define robot motors
    motor_1 = robot.getDevice("left_wheel_motor")
    motor_2 = robot.getDevice("right_wheel_motor")
    # motor_3 = robot.getDevice("motor_3")  

    # set position for robot motors
    motor_1.setPosition(float('inf'))
    motor_2.setPosition(float('inf'))
    # motor_3.setPosition(float('inf'))

    # set velocity for robot motors
    motor_1.setVelocity(0.0)
    motor_2.setVelocity(0.0)
    # motor_3.setVelocity(0.0)
    
    # print("Motor 1 Velocity ", motor_1.getVelocity())
    # print("Motor 2 Velocity ", motor_2.getVelocity())


    # define position sensors
    pos_1 = robot.getDevice("left_wheel_sensor")
    pos_2 = robot.getDevice("right_wheel_sensor")
    # pos_3 = robot.getDevice("pos_3")  

    # enable position sensors
    pos_1.enable(time_step)
    pos_2.enable(time_step)
    # pos_3.enable(time_step)
    # print("Wheel Sensors ", pos_1, pos_2)

    # define infra red sensors
    ir_1 = robot.getDevice("ps0")
    ir_2 = robot.getDevice("ps1")
    ir_3 = robot.getDevice("ps2")
    ir_4 = robot.getDevice("ps3")
    ir_5 = robot.getDevice("ps4")
    ir_6 = robot.getDevice("ps5")
    ir_7 = robot.getDevice("ps6")
    ir_8 = robot.getDevice("ps7")

    # enable infra red sensors
    ir_1.enable(time_step)
    ir_2.enable(time_step)
    ir_3.enable(time_step)
    ir_4.enable(time_step)
    ir_5.enable(time_step)
    ir_6.enable(time_step)
    ir_7.enable(time_step)
    ir_8.enable(time_step)

    # print("IR Sensors ", ir_1, ir_2, ir_3, ir_4, ir_5, ir_6, ir_7, ir_8)


    wheel_cirum = 2 * math.pi * WHEEL_RADIUS
    encoder_unit = wheel_cirum / (2*math.pi)
    # print("Wheel Circumference: ", wheel_cirum)
    # print("Encoder Unit: ", encoder_unit)

    # define and enable lidar sensors
    # lidar = robot.getDevice("head_hokuyo_sensor");
    # lidar.enable(time_step);

    # define and enable gps
    gps = robot.getDevice("gps")
    gps.enable(time_step)
    # print("GPS Sensor ", gps)

    # define and enable compass
    compass = robot.getDevice("compass")
    compass.enable(time_step)
    # print("Compass Sensor ", compass)

    imu = robot.getDevice("inertial_unit")
    imu.enable(time_step)

    print("Initializing...")

    return robot, goal_pos, start_pos


def read_sensors_values():

    global gps_values
    global compass_val
    global position_value
    global ir_value
    global imu_yaw

    # read GPS values
    gps_values = gps.getValues()
    # print("GPS Read: ", gps_values[0], gps_values[1], gps_values[2])

    # read compass and rotate arrow accordingly
    compass_val = compass.getValues()
    # print("Compass Read: ", compass_val)

    # read position sensors values
    position_value = encoder_unit*np.array([pos_1.getValue(),pos_2.getValue()])
    # print("Position Read: ", position_value)
    
    # read infra-red sensors values
    ir_value = np.array([ir_1.getValue(),ir_2.getValue(), ir_3.getValue(), ir_4.getValue(), ir_5.getValue(), ir_6.getValue(), ir_7.getValue(), ir_8.getValue()])
    # print("IR Read: ", ir_value)

    # read the z-axis rotation from the yaw value
    imu_rpy = imu.getRollPitchYaw()
    imu_yaw = math.degrees(imu_rpy[2])
    if(imu_yaw < 0):
       imu_yaw += 360
    elif(imu_yaw > 360):
       imu_yaw -= 360
    # print("Orientation: ", imu_yaw)

    # print("Sensor Read Complete")
    return gps_values,compass_val,position_value,ir_value, imu_yaw

def init_robot_state(in_pos=robot_position,in_omega=robot_omega):
    global robot_position
    global robot_omega
    # define robot state here
    robot_position = in_pos
    robot_omega    = in_omega

def update_robot_state():
    global robot_velocity
    global robot_position
    global robot_omega
    # updating the current theta
    robot_position[2] = math.atan2(compass_val[0], compass_val[1])
    
    # updating the currnet robot position
    robot_position[0] = gps_values[0]
    robot_position[1] = gps_values[1]
    # print("Robot Position Compass: ", robot_position[2])
    # print("Robot Position [1]: ", robot_position[1])
    # print("Robot Position [0]: ", robot_position[0])

def update_motor_speed(input_omega=robot_omega):
    motor_1.setVelocity(input_omega[0])
    motor_2.setVelocity(input_omega[1])
    # motor_3.setVelocity(input_omega[2])
    # print("Motor 1 Speed Updated to: ", input_omega[0])
    # print("Motor 2 Speed Updated to: ", input_omega[1])

def get_north_bearing_in_degrees(north):
  rad = math.atan2(north[0], north[1])
  #print(rad)
  bearing = math.degrees(rad)
  #print(bearing)
  if (bearing < 0.0):
    bearing = bearing + 360.0
  elif (bearing > 360):
     bearing = bearing - 360
  # print("bearing: ", bearing)
  
  return bearing