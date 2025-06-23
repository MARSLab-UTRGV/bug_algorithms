# Multiple_Maps_Spawner_Controller.py

from controller import Supervisor
import os

# ==============================================================================
# --- MAP CONFIGURATION DICTIONARY (Unchanged) ---
# ==============================================================================
MAP_CONFIGURATIONS = {
    "Easy Maze Map 1 (Fig. 9 From Paper) v1.obj": {"robot": [-.0195, .827, 0], "goal": [-.35, .01, 0]},
    "Easy Maze Map 2 (Fig. 3 From Paper) v2.obj": {"robot": [0.035482, 0.0266135, 0], "goal": [0.01, 1.07, 0]},
    "Easy Maze Map 3 (Fig. 12 From Paper) v1.obj": {"robot": [-0.624459, 0.597473, 0], "goal": [0.51, -.5, 0]},
    "Medium Maze Map (Fig. 4 From Paper) v2.obj": {"robot": [-0.0012222, 1.24371, 0], "goal": [0, .05, 0]},
    "Medium Maze Map 2 (Fig. 10 From Paper) v3.obj": {"robot": [-0.559898, 0.549806, 0], "goal": [0.51, -0.42, 0]},
    "Medium Maze Map 3 (Fig. 15 From Paper) v2.obj": {"robot": [-0.668768, 0.541166, 0], "goal": [0.44, -0.55, 0]},
    "Hard Maze Map 1 (Fig. 1 From Paper) v2.obj": {"robot": [0.9564, -0.977176, 0], "goal": [-0.77, 0.76, 0]},
    "Hard Maze Map 2 (Fig B.17 From Paper) v1.obj": {"robot": [-0.878284, 0.871113, 0], "goal": [0.87, -0.88, 0]},
    "Hard Maze Map 3 (Fig. 7 From Paper) v2.obj": {"robot": [-0.920181, 0.929516, 0], "goal": [0.86, -0.18, 0]}
}
DEFAULT_CONFIG = {"robot": [0, 0, 0.05], "goal": [0.5, 0.5, 0.05]}

# ==============================================================================

try:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
    CAD_FILES_DIRECTORY = os.path.join(project_root, 'CadFiles')
    if not os.path.isdir(CAD_FILES_DIRECTORY):
        raise FileNotFoundError
except FileNotFoundError:
    print("FATAL ERROR: Could not automatically determine the 'CadFiles' directory path.")
    CAD_FILES_DIRECTORY = ""

print(f"[Supervisor]: Controller started. Using CAD files from: {CAD_FILES_DIRECTORY}")

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

robot_node_self = supervisor.getSelf()
if robot_node_self:
    robot_def_name = robot_node_self.getDef()
    window_field = robot_node_self.getField("window")
    if window_field:
        window_name = window_field.getSFString()
        if window_name:
            url = f"http://localhost:1234/robot_windows/{window_name}/{window_name}.html?name={robot_def_name}"
            print(f"\n--- Robot UI is available at: {url} ---\n")

root_node = supervisor.getRoot()
children_field = root_node.getField('children')

def clear_current_maze():
    print("[Supervisor]: Searching for and removing old maze...")
    current_maze_node = supervisor.getFromDef("CURRENT_MAZE")
    if current_maze_node:
        current_maze_node.remove()
        print("[Supervisor]: Previous maze found and removed.")
    else:
        print("[Supervisor]: No previous maze to remove.")

# --- NEW FUNCTION TO MANAGE THE TRAIL GROUP ---
def create_or_clear_trail_group():
    """Finds the trail group and clears its children, or creates it if it doesn't exist."""
    trail_group_node = supervisor.getFromDef("TRAIL_GROUP")
    
    if trail_group_node:
        # If the group exists, clear all its children (the old trail spheres)
        print("[Supervisor]: Clearing old trail spheres...")
        trail_children_field = trail_group_node.getField("children")
        # Loop until the group is empty
        while trail_children_field.getCount() > 0:
            trail_children_field.removeMF(0) # Remove the first child
    else:
        # If it's the first run, create the empty group at the world root
        print("[Supervisor]: Creating trail group for the first time.")
        group_string = "DEF TRAIL_GROUP Group {}"
        children_field.importMFNodeFromString(-1, group_string)

def spawn_maze(obj_filename):
    if not CAD_FILES_DIRECTORY:
        print("[Supervisor]: ERROR: CAD_FILES_DIRECTORY is not set. Aborting spawn.")
        return
    
    clear_current_maze()
    create_or_clear_trail_group() # Call the new function to clear the trail
    
    obj_full_path = os.path.join(CAD_FILES_DIRECTORY, obj_filename)
    webots_path = obj_full_path.replace(os.path.sep, '/')

    if not os.path.exists(obj_full_path):
        print(f"[Supervisor]: ERROR! Cannot find maze file at '{obj_full_path}'. Spawn aborted.")
        return

    print(f"[Supervisor]: Spawning STATIC maze from: {webots_path}")

    maze_string = f"""
        DEF CURRENT_MAZE Solid {{
          translation 0 0 0
          rotation 0 0 1 0
          children [ Shape {{ appearance PBRAppearance {{ baseColor 1 1 1 metalness 0 }} geometry Mesh {{ url [ "{webots_path}" ] }} }} ]
          boundingObject Mesh {{ url [ "{webots_path}" ] }}
        }} """
    
    children_field.importMFNodeFromString(-1, maze_string)
    print(f"[Supervisor]: Successfully spawned '{obj_filename}'.")
    reposition_robot_and_goal(obj_filename)

def reposition_robot_and_goal(map_name):
    """Places the robot and goal and restarts the robot's controller."""
    robot_node = supervisor.getFromDef("bug")
    goal_node = supervisor.getFromDef("goal")
    
    config = MAP_CONFIGURATIONS.get(map_name, DEFAULT_CONFIG)
    if map_name not in MAP_CONFIGURATIONS:
        print(f"[Supervisor]: WARNING! No specific configuration found for '{map_name}'. Using default positions.")

    robot_pos = config["robot"]
    goal_pos = config["goal"]

    if robot_node and goal_node:
        robot_trans_field = robot_node.getField("translation")
        goal_trans_field = goal_node.getField("translation")
        if robot_trans_field and goal_trans_field:
            robot_trans_field.setSFVec3f(robot_pos)
            goal_trans_field.setSFVec3f(goal_pos)
            robot_node.resetPhysics()
            print(f"[Supervisor]: Robot and goal repositioned for '{map_name}'.")

            # Restart the robot's controller to reset its logic
            robot_node.restartController()
            print("[Supervisor]: Robot controller has been restarted.")

# --- Main Simulation Loop ---
print("\n[Supervisor]: Initializing simulation...")
spawn_maze("Easy Maze Map 1 (Fig. 9 From Paper) v1.obj")
print("[Supervisor]: Initialization Complete. Waiting for commands from UI...\n")

while supervisor.step(timestep) != -1:
    message = supervisor.wwiReceiveText()
    if message:
        print(f"\n[Supervisor]: SUCCESS! Received command from UI: '{message}'")
        spawn_maze(message)