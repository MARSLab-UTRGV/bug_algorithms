"""bug0_algo controller.
1) head toward goal 
2) follow obstacles until you can head toward the goal again 
3) continue
"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
###IMPORTS#######################################
from math import fabs
from re import S
import sys, os, time
import csv
from tkinter import SE
from controller import Robot, Supervisor
from collections import deque

try: 
    from csv import DictWriter
    import numpy as np
    import math
except ImportError:
    sys.exit("Warning: modules not found.")
#################################################




###Object-Class for Epuck(Target)
class EpuckBug(Supervisor):
    #epuck constructor
    def __init__(self, 
                 epuck_params,
                 time_step, 
                 files_dict):
        #using Robot class
        Robot.__init__(self)
        #init timestep
        try:
            self.time_step = time_step
        except:
            self.time_step = int(self.getBasicTimeStep())
        print("epuck using timestep: {}ms".format(self.time_step))
        
        #self-dictionary for files
        self.files_dict = files_dict

        #initiate states-->init-state=move to goal
        self.bot_states = {'A': 'move_to_goal',
                           'B': 'follow_wall',
                           'C': 'empty',
                           'D': 'reached_goal'}
        #initiate epuck as moving to goal
        self.bot_state = self.bot_states['A']
        #print("epuck state init'd and grounded...")

        #movement stack
        self.tgt_mvmt_stack = deque()

        # Get and enable devices.
        #gps device
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        #compass device
        self.compass = self.getDevice("compass")
        self.compass.enable(self.time_step)
        #accelerometer device
        self.accelerometer = self.getDevice("accelerometer")
        self.accelerometer.enable(self.time_step)
        #gyro device
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)
        #imu device
        self.imu = self.getDevice("inertial_unit")
        self.imu.enable(self.time_step)

        # define infra red sensors
        self.ir_1 = self.getDevice("ps5")
        self.ir_2 = self.getDevice("ps6")
        self.ir_3 = self.getDevice("ps7")
        self.ir_4 = self.getDevice("ps0") 
        self.ir_5 = self.getDevice("ps1")
        self.ir_6 = self.getDevice("ps2")

        # enable infra red sensors
        self.ir_1.enable(time_step)
        self.ir_2.enable(time_step)
        self.ir_3.enable(time_step)
        self.ir_4.enable(time_step)
        self.ir_5.enable(time_step)
        self.ir_6.enable(time_step)

        # Motor instance to drive robot
        #left motor and wheel
        self.left_motor = self.getDevice('left_wheel_motor')
        self.left_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.left_wheel_pos = self.getDevice('left_wheel_sensor')
        self.left_wheel_pos.enable(self.time_step)
        
        #right motor and wheel
        self.right_motor = self.getDevice('right_wheel_motor')
        self.right_motor.setPosition(float('inf'))
        self.right_motor.setVelocity(0.0)
        self.right_wheel_pos = self.getDevice('right_wheel_sensor')
        self.right_wheel_pos.enable(self.time_step)
                
        #set encoder unit
        wheel_radius = float(epuck_params["epuck_wheel_radius"])
        wheel_circumference = 2 * math.pi * wheel_radius
        self.encoder_unit = wheel_circumference / (2 * math.pi)
        
        #set omega - wheel angular velocity
        self.omega = [0, 0]
        
        #self.current_pose = 6 * [0]  # X, Y, Z
        self.current_pose = [0, 0, 0]
        #including previous-pose to calculate velocity
        self.previous_pose = self.current_pose
        #target waypoint dist and threshold
        self.target_waypt = [0, 0, 0]
        self.target_waypt_thresh = float(epuck_params["waypoint_reached_tolerance"])
        self.tgt_distance = 0.0
        #target heading and tolerance
        self.tgt_heading = 0
        self.target_heading_tol = float(epuck_params["waypoint_heading_tolerance"])
        #max wheel speed
        self.max_speed  = float(epuck_params["epuck_max_fwd_speed"])
        self.wall_Kp = float(epuck_params["wall_dist_proportional_val"])
        self.wall_tgt_dist = float(epuck_params["wall_tgt_distance"])
        #obstacle hit point
        self.hit_point = [0, 0]
        self.hit_point_timer = 0
        #obstacle leave point
        self.leave_point  = [0, 0]
        self.leave_pt_tgt_dist = 0.0
        #obstacle final leave point
        self.final_leave_point = [0, 0]
        self.final_leave_pt_tgt_dist = 0.0
        
        print("Epuck initialized")

    
    #clamp-fcn: return min-val b/w input max value
    #and (the max b/w curr val and min val)
    def clamp(self, value, value_min, value_max):
        return min(max(value, value_min), value_max)


    #set position for robot
    def set_position(self, pos):
        """
        Set the new absolute position of the robot
        Parameters:
            pos (list): [X,Y,Z] current absolute position and angles
        """
        self.current_pose = pos
        

    #set previous position of robot
    def set_prev_position(self, pos):
        self.previous_pose = pos


    def read_sensors_values(self):
        # read GPS values
        gps_values = self.gps.getValues()
        #print("GPS --> x: {:.4f} y: {:.4f}, z: {}".format(gps_values[0], gps_values[1], gps_values[2]))

        # read compass and rotate arrow accordingly
        #compass_val = self.compass.getValues()
        #print("Compass Read: {}".format(compass_val))
        
        # read imu values
        imu_val = self.imu.getRollPitchYaw()[2]

        # read position sensors values
        position_value = self.encoder_unit*np.array([self.left_wheel_pos.getValue(),
                                                     self.right_wheel_pos.getValue()])
        #print("Position Read: {}".format(position_value))
    
        # read infra-red sensors values
        ir_value = np.array([self.ir_1.getValue(),
                             self.ir_2.getValue(), 
                             self.ir_3.getValue(), 
                             self.ir_4.getValue(), 
                             self.ir_5.getValue(), 
                             self.ir_6.getValue()])
        #print("IR Read: {}".format(ir_value))

        #print("Sensor Read Complete")
        return gps_values, imu_val, position_value, ir_value


    #update robot state
    def update_robot_state(self, gps_vals, imu_val):
        # updating the current theta/bearing for epuck
        #theta = math.atan2(compass_vals[0], compass_vals[1])
        #bearing = (theta - (math.pi / 2)) / math.pi * 180.0
        #if bearing < 0:
        #    bearing += 360
        
        bearing = round(imu_val, 4)
    
        # updating the currnet robot position
        x_pos = round(gps_vals[0], 4)
        y_pos = round(gps_vals[1], 4)
        curr_pose = [x_pos, y_pos, bearing]
        self.set_position(curr_pose)
        print("current pose: {}".format(self.current_pose))


    def set_motor_speeds(self):
        #set motor speeds
        self.left_motor.setVelocity(self.omega[0])
        self.right_motor.setVelocity(self.omega[1])
        #print("motor speeds set: {}".format(self.omega))
        

    def get_tgt_distance(self):
        return math.sqrt((self.current_pose[0] - self.target_waypt[0])**2 + (self.current_pose[1] - self.target_waypt[1])**2)
    

    #return tgt-bearing to current given waypoint
    def getTgtBearing(self):
        #calculate tgt-bearing
        # This will be in ]-pi;pi]
        angle_diff = math.atan2(self.target_waypt[1] - self.current_pose[1], 
                               self.target_waypt[0] - self.current_pose[0])
        #print("angle diff: {:.4f}".format(angle_diff))
        
        #return angle_left
        return angle_diff


    #writes state for mavic2pro/robot-node
    def writeBotState(self, input_arr, filename):
        field_names = ['x_pos', 
                       'y_pos', 
                       'bearing',
                       'timestep']

        csv_dict = {'x_pos': input_arr[0],
                    'y_pos': input_arr[1],
                    'bearing': input_arr[2],
                    'timestep': input_arr[3]}

        with open(filename, 'a', newline='', encoding='utf-8') as f_obj:
            dictWriter_obj = DictWriter(f_obj, fieldnames=field_names)
            try:
                dictWriter_obj.writerow(csv_dict)
            except:
                print("sim crashed on writing state...")
                self.cleared_crs = True
            f_obj.close()


    #running quadrotor with given params
    def run(self, params):
        t1 = self.getTime()
        #calcd_time_step = (self.time_step * 4) / 1000
        calcd_time_step = self.time_step / 1000
        print("using calcd time step for PIDs: {}".format(calcd_time_step))
        
        #collect world gravity
        world_grav = params['world_gravity']
        #print("gravity: {}m/s^2".format(world_grav))
        
        #collect target waypoint
        try:
            target_node = self.getFromDef("goal")
            self.target_waypt = target_node.getField("translation").getSFVec3f()
            print("target waypoint: {}".format(self.target_waypt))
        except:
            print("Target node not found...")
            
        #start main sim loop
        sim_start_time = self.getTime()
        #set previous time
        prevTime = float(0)
        print("sim_start_time: {}".format(sim_start_time))

        while self.step(self.time_step) != -1:

            #collect sensor values
            gps_values, imu_val, encoder_value, ir_value = self.read_sensors_values()
            #print("Sensor Read Complete")
            front_ir_val = np.mean(np.array([ir_value[2], ir_value[3]]))
            right_ir_val = np.mean(np.array([ir_value[4], ir_value[5]]))
            left_ir_val = np.mean(np.array([ir_value[0], ir_value[1]]))
            
            #obstacle boolean array
            #obstacle_bool = [np.mean(left_ir_values) > 80,
            #                 np.mean(front_ir_values) > 80,
            #                 np.mean(right_ir_values) > 80]
            
            #update robot state
            self.update_robot_state(gps_values, imu_val)
            
            #get target distance
            self.tgt_distance = self.get_tgt_distance()
            print("target dist: {:.4f}".format(self.tgt_distance))
            
            #get target bearing
            self.tgt_heading = self.getTgtBearing()
            
            #get bearing to goal
            angle_diff = self.tgt_heading - self.current_pose[2]
            #print("angle diff: {:.4f}".format(angle_diff))
            
            #if bot still not near goal
            if self.tgt_distance < self.target_waypt_thresh:
                print("reached waypoint...")
                self.omega = [0.0, 0.0]
                self.bot_state = 'reached_goal'
            
            #if in reached -goal state, stop
            if self.bot_state == 'reached_goal':
                print("reached goal...")
                self.simulationSetMode(0)
            
            #else if bot is moving to goal
            elif self.bot_state == 'move_to_goal':

                print("In move to goal state...")
                
                #if bot is not facing goal-->turn to face goal
                if abs(angle_diff) > self.target_heading_tol:
                    #if angle_diff is negative, turn left
                    if angle_diff < 0:
                        #turn right/CW
                        self.omega = [0.2 * self.max_speed, -0.2 * self.max_speed]
                    else:
                        #turn left/CCW
                        self.omega = [-0.2 * self.max_speed, 0.2 * self.max_speed]
                else:
                    # Move forward
                    self.omega = [self.max_speed, self.max_speed]

                if front_ir_val > 80 or left_ir_val > 80 or right_ir_val > 80:
                    #record hit point
                    if self.hit_point[0] == 0.0 and self.hit_point[1] == 0.0:
                        self.hit_point = [round(self.current_pose[0], 4),
                                          round(self.current_pose[1], 4)]
                        self.leave_pt_tgt_dist = self.get_tgt_distance()
                        print("initial hit point tgt-dist: {:.4f}".format(self.leave_pt_tgt_dist))
                        self.hit_point_timer = self.getTime()
                        print("time obstacle hit: {:.4f}".format(self.hit_point_timer))
                    print("hit point at: x: {:.4f}, y: {:.4f}".format(self.hit_point[0],
                                                                      self.hit_point[1]))
                    #swap to wall-following state
                    self.bot_state = self.bot_states["B"]

                
            #if in wall following state
            elif self.bot_state == 'follow_wall':
                
                print("In wall following state...")
                #error distance between wall and epuck
                wall_error = right_ir_val - self.wall_tgt_dist
                
                #printing proximity sensors
                #print("left: {:.4f} front avg: {:.4f} right sens avg: {:.4f}".format(left_ir_val,
                #                                                                     front_ir_val,
                #                                                                     right_ir_val))
                
                #if obstacle ahead, turn CCW
                if left_ir_val > 80 or front_ir_val > 80:
                    self.omega = [-self.max_speed, 
                                  self.max_speed]
                
                #else if right value detects obstacle, follow wall
                elif right_ir_val > 80:
                    self.omega = [self.max_speed - self.wall_Kp * wall_error, 
                                  self.max_speed + self.wall_Kp * wall_error]
                
                #else turn to follow wall
                else:
                    self.omega = [self.max_speed, 
                                  -self.max_speed]
 
                #collect current tgt-distance
                curr_tgt_distance = self.get_tgt_distance()
                print("current tgt dist: {:.4f}".format(curr_tgt_distance))

                #compare current tgt-dist with distance from hit point
                if curr_tgt_distance < self.leave_pt_tgt_dist:
                    self.leave_point = [round(self.current_pose[0], 4),
                                        round(self.current_pose[1], 4)]
                    self.leave_pt_tgt_dist = self.get_tgt_distance()
                print("current leave distance: {:.4f}".format(self.leave_pt_tgt_dist))
                
                

                #if we have reached leave point --> set target-waypoint to goal
                #if self.target_waypt == self.leave_point:
                #    if self.current_pose[0] == self.leave_point[0] and self.current_pose[1] == self.leave_point[1]:
                #        self.hit_point = [0, 0]
                #        self.target_waypt = target_node.getField("translation").getSFVec3f()

                #if we reached our hit point-->set target-waypoint to leave point
                hit_point_dist = math.sqrt((self.current_pose[0] - self.hit_point[0])**2 + (self.current_pose[1] - self.hit_point[1])**2)
                print("hit_point_dist: {:.4f}   time_diff: {}".format(hit_point_dist,
                                                                      abs(self.getTime() - self.hit_point_timer)))
                if hit_point_dist < 0.05:
                    if abs(self.getTime() - self.hit_point_timer) > 5 and hit_point_dist < 0.05:
                        print("ran into our hit point...")
                        self.final_leave_point = [self.leave_point[0], self.leave_point[1]]
                        self.final_leave_pt_tgt_dist = self.leave_pt_tgt_dist
                    

                print("current hit-point: x: {:.4f}, y: {:.4f}".format(self.hit_point[0],
                                                                       self.hit_point[1]))
                print("current leave-point x: {:.4f}, y: {:.4f}".format(self.leave_point[0],
                                                                        self.leave_point[1]))
                print("final leave distance: {:.4f}".format(self.final_leave_pt_tgt_dist))
                print("final leave-point x: {:.4f}, y: {:.4f}".format(self.leave_point[0],
                                                                        self.leave_point[1]))
                
                print("Curr: ", curr_tgt_distance)
                print("Final: ", self.final_leave_pt_tgt_dist)
                print("Curr-final: ", curr_tgt_distance-self.final_leave_pt_tgt_dist)

                if abs(curr_tgt_distance-self.final_leave_pt_tgt_dist) < 0.05 and self.final_leave_pt_tgt_dist != 0:
                    print("Back at leave point")
                    self.bot_state = self.bot_states["A"]

                




            else:
                print("in lost state...")
                x_dist = self.current_pose[0] - self.hit_point[0]
                y_dist = self.current_pose[1] - self.hit_point[1]
                print("x_dist: {:.4f}  y_dist: {:.4f}".format(x_dist,
                                                              y_dist))



                #else if there are no obstacles detected, swap to move to target state
                #elif front_ir_val < 80 and left_ir_val < 80 and right_ir_val < 80:
                #    self.bot_state = self.bot_states["A"]

                
                    
                
                

                
                
                
            


            
            #collect bot-pose for file
            bot_state = [self.current_pose[0], 
                         self.current_pose[1],
                         self.current_pose[2],
                         self.getTime()]
            
            #write bot-pose to file
            #self.writeBotState(bot_state, self.files_dict['state-filedir'])
                         
            
            #update motor speeds
            self.set_motor_speeds()

            




#clear file, if exists
def clearFileIfExists(filename):
    if os.path.isfile(filename):
        os.remove(filename)


#retreive params from particle csvfile
def getParams(params_file):
    params_dict = dict()
    while not os.path.exists(params_file):
        print("waiting for file to be written...")
        time.sleep(1)

    if os.path.isfile(params_file):
        with open(params_file, "r") as f:
            lines = csv.reader(f)
            for line in lines:
                params_dict[line[0]] = line[1]
            f.close()
    else:
        raise ValueError("%s isn't a file!".format(params_file))
    return params_dict


def main():
    #init print
    print("numpy version: {}".format(np.__version__))
    
    #create main-parent directory
    main_dir = os.path.abspath(os.path.join(__file__, "../../.."))
    #file directory for simulation parameters
    #default parameters
    param_filedir = main_dir + r"\controllers\bug1\params_edit.csv"
    #state filedire
    state_filename = main_dir + r"\python_utilities\epuck_state_bug1.csv"
    clearFileIfExists(state_filename)

    ###experimental-designator filename
    files_dict = {'param-filedir': param_filedir,
                  'state-filedir': state_filename}

    #collect parameters for sim
    epuck_params = getParams(param_filedir)
    #print params
    for key, val in epuck_params.items():
        print("{}: {}".format(key, val))
    #establish timestep and takeoff thresh constants
    TIME_STEP = int(epuck_params["EPUCK_TIME_STEP"])
    
    #construct robot
    robot = EpuckBug(epuck_params, 
                     TIME_STEP, 
                     files_dict)
    robot.run(epuck_params)



if __name__ == "__main__":
    main()


    