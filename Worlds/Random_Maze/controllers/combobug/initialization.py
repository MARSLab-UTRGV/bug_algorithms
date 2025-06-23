from controller import GPS
from controller import DistanceSensor
from controller import PositionSensor
from controller import Node
from controller import Robot
from controller import Supervisor
import math
import numpy as np

WHEEL_RADIUS = 0.029
CHASSIS_AXLE_LENGTH = 0.22

wheel_cirum = 2 * math.pi * WHEEL_RADIUS
encoder_unit = wheel_cirum / (2*math.pi)

# --- Global variables for devices ---
robot = None
gps = None
compass = None
imu = None
motor_1 = None
motor_2 = None
pos_1 = None
pos_2 = None
ir_sensors = []

# --- Global variables for sensor readings ---
gps_values = None
compass_val = None
position_value = None
ir_value = None
imu_yaw = None

def init_devices(time_step=32):
    """
    Initializes the robot and its devices. This should be called first.
    """
    global robot, gps, compass, imu, motor_1, motor_2, pos_1, pos_2, ir_sensors

    # Create the Robot instance.
    robot = Supervisor()

    # Define robot motors
    motor_1 = robot.getDevice("left_wheel_motor")
    motor_2 = robot.getDevice("right_wheel_motor")
    motor_1.setPosition(float('inf'))
    motor_2.setPosition(float('inf'))
    motor_1.setVelocity(0.0)
    motor_2.setVelocity(0.0)

    # Define and enable position sensors
    pos_1 = robot.getDevice("left_wheel_sensor")
    pos_2 = robot.getDevice("right_wheel_sensor")
    pos_1.enable(time_step)
    pos_2.enable(time_step)

    # Define and enable infrared sensors
    ir_sensors = []
    for i in range(8):
        sensor = robot.getDevice(f"ps{i}")
        sensor.enable(time_step)
        ir_sensors.append(sensor)

    # Define and enable GPS, Compass, and IMU
    gps = robot.getDevice("gps")
    gps.enable(time_step)
    compass = robot.getDevice("compass")
    compass.enable(time_step)
    imu = robot.getDevice("inertial_unit")
    imu.enable(time_step)

    print("Devices Initialized...")
    return robot

def get_world_state(robot_supervisor):
    """
    Gets the start and goal positions from the world.
    This should be called *after* the supervisor has set up the maze.
    """
    goal_node = robot_supervisor.getFromDef('goal')
    if not goal_node:
        # Fallback in case the supervisor renamed the goal
        goal_node = robot_supervisor.getFromDef('GOAL_READY')
    
    robot_node = robot_supervisor.getFromDef('bug')

    if not goal_node or not robot_node:
        print("Error: Could not find 'goal' or 'bug' DEF nodes in the world.")
        return None, None
    
    goal_pos_3d = goal_node.getField('translation').getSFVec3f()
    start_pos_3d = robot_node.getField('translation').getSFVec3f()

    return start_pos_3d, goal_pos_3d

def read_sensors_values():
    """Reads all sensor values for the current timestep."""
    global gps_values, compass_val, position_value, ir_value, imu_yaw

    # Read GPS values
    gps_values = gps.getValues()

    # Read compass values
    compass_val = compass.getValues()

    # Read position sensors values
    position_value = encoder_unit * np.array([pos_1.getValue(), pos_2.getValue()])
    
    # Read infra-red sensors values
    ir_value = np.array([sensor.getValue() for sensor in ir_sensors])

    # Read the z-axis rotation from the yaw value
    imu_rpy = imu.getRollPitchYaw()
    imu_yaw = math.degrees(imu_rpy[2])
    if imu_yaw < 0:
       imu_yaw += 360
    elif imu_yaw > 360:
       imu_yaw -= 360

    return gps_values, compass_val, position_value, ir_value, imu_yaw

def update_motor_speed(input_omega):
    """Sets the velocity for the left and right motors."""
    motor_1.setVelocity(input_omega[0])
    motor_2.setVelocity(input_omega[1])

def draw_robot_path(robot_pos, color_str="1 0.7 0.1", transparency="0.5", parent_node=None):
    """
    Creates a small sphere to mark the robot's path.
    The robot controller must have supervisor TRUE to use this.
    """
    # --- THIS IS THE FIX ---
    # The 'parent_node' argument was missing from the function definition.
    if parent_node is None:
        parent_node = robot.getRoot()
    
    children_field = parent_node.getField("children")
    
    # Create the VRML string for a small, transparent sphere.
    path_point_string = f'Transform {{ translation {robot_pos[0]} {robot_pos[1]} 0.002 children [ Shape {{ appearance PBRAppearance {{ baseColor {color_str} transparency {transparency} metalness 0 }} geometry Sphere {{ radius 0.015 subdivision 8 }} }} ] }}'
    
    children_field.importMFNodeFromString(-1, path_point_string)

# Deprecated/Unused functions below, can be removed but kept for reference
def get_north_bearing_in_degrees(north):
  rad = math.atan2(north[0], north[1])
  bearing = math.degrees(rad)
  if (bearing < 0.0):
    bearing = bearing + 360.0
  elif (bearing > 360):
     bearing = bearing - 360
  return bearing

def init_robot_state(in_pos=None,in_omega=None):
    pass

def update_robot_state():
    pass

def init_robot(time_step=32):
    print("Warning: init_robot() is deprecated. Use init_devices() and get_world_state().")
    robot_supervisor = init_devices(time_step)
    start_pos, goal_pos = get_world_state(robot_supervisor)
    return robot_supervisor, goal_pos, start_pos