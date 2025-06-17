/*
 * Controller for a Webots Supervisor robot that spawns objects and mazes
 * based on messages from an HTML robot window.
 * FINAL VERSION: Implements seamless post-and-wall logic for both the
 * maze and the solution path for a perfect visual result.
 */

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/plugins/robot_window/robot_wwi.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>

#define TIME_STEP 64

// ANSI COLOR CODES
#define ANSI_COLOR_RESET   "\x1b[0m"
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"

// --- MAZE GEOMETRY & LAYOUT ---
#define GRID_SIZE 10
#define CELL_SIZE 0.3
#define WALL_THICKNESS 0.05
#define WALL_HEIGHT 0.075
#define WALL_Z_POS (WALL_HEIGHT / 2.0)
#define MAZE_OFFSET (-(GRID_SIZE * CELL_SIZE) / 2.0)
// --- NEW: Corrected length for maze walls to fit between posts ---
#define MAZE_WALL_LENGTH (CELL_SIZE - WALL_THICKNESS)

// --- PATH VISUALIZATION SETTINGS ---
#define PATH_LINE_THICKNESS 0.03
#define PATH_HEIGHT 0.005
#define PATH_Z_POS (WALL_HEIGHT - (PATH_HEIGHT / 2.0))
#define PATH_WALL_LENGTH (CELL_SIZE - PATH_LINE_THICKNESS)

// --- VRML TEMPLATES ---
char POST_TEMPLATE[256];
char WALL_TEMPLATE[512]; // Increased size for new format specifier
char PATH_POST_TEMPLATE[512];
char PATH_WALL_TEMPLATE[512];

WbNodeRef maze_parent_node_ref = NULL;
typedef struct { int x; int y; } Point;
Point passages[GRID_SIZE * GRID_SIZE * 2];
int num_passages = 0;
bool visited_gen[GRID_SIZE][GRID_SIZE];
Point path_solution[GRID_SIZE * GRID_SIZE];
int path_length = 0;

// --- FUNCTION DECLARATIONS ---
void initialize_vrml_templates();
void generate_maze_passages_c(int n);
void find_shortest_path_bfs();
void clear_current_maze();
void build_and_spawn_maze();

void initialize_vrml_templates() {
    snprintf(POST_TEMPLATE, sizeof(POST_TEMPLATE), "Solid { translation %%f %%f %f children [ Shape { geometry Box { size %f %f %f } appearance PBRAppearance { baseColor 0.1 0.1 0.1 } } ] boundingObject Box { size %f %f %f } }", WALL_Z_POS, WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT, WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT);
    
    // MODIFIED: Wall template now uses a format specifier for its length
    snprintf(WALL_TEMPLATE, sizeof(WALL_TEMPLATE), "Solid { translation %%f %%f %f rotation 0 0 1 %%f children [ Shape { geometry Box { size %%f %f %f } appearance PBRAppearance { baseColor 0.1 0.1 0.1 } } ] boundingObject Box { size %%f %f %f } }", WALL_Z_POS, WALL_THICKNESS, WALL_HEIGHT, WALL_THICKNESS, WALL_HEIGHT);

    // MODIFIED: Brighter red path with explicit non-metallic appearance
    char appearance_string[256];
    snprintf(appearance_string, sizeof(appearance_string), "appearance PBRAppearance { baseColor 1 0 0 metalness 0 transparency 0.3 }");

    snprintf(PATH_POST_TEMPLATE, sizeof(PATH_POST_TEMPLATE), "Transform { translation %%f %%f %f children [ Shape { %s geometry Box { size %f %f %f } } ] }", PATH_Z_POS, appearance_string, PATH_LINE_THICKNESS, PATH_LINE_THICKNESS, PATH_HEIGHT);
    snprintf(PATH_WALL_TEMPLATE, sizeof(PATH_WALL_TEMPLATE), "Transform { translation %%f %%f %f rotation 0 0 1 %%f children [ Shape { %s geometry Box { size %f %f %f } } ] }", PATH_Z_POS, appearance_string, PATH_WALL_LENGTH, PATH_LINE_THICKNESS, PATH_HEIGHT);
}

// (generate_maze and has_wall logic is unchanged)
void clear_current_maze() { if (maze_parent_node_ref) { wb_supervisor_node_remove(maze_parent_node_ref); maze_parent_node_ref = NULL; } }
bool has_wall_c(Point p1, Point p2) { for (int i = 0; i < num_passages; i += 2) { if ((passages[i].x == p1.x && passages[i].y == p1.y && passages[i+1].x == p2.x && passages[i+1].y == p2.y) || (passages[i].x == p2.x && passages[i].y == p2.y && passages[i+1].x == p1.x && passages[i+1].y == p1.y)) return false; } return true; }
void generate_maze_passages_c(int n) { num_passages = 0; for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) visited_gen[i][j] = false; Point stack[n * n]; int stack_top = 0; stack[stack_top++] = (Point){0, 0}; visited_gen[0][0] = true; int dx[] = {0, 0, 1, -1}, dy[] = {1, -1, 0, 0}; while (stack_top > 0) { Point current = stack[stack_top - 1]; Point neighbors[4]; int num_neighbors = 0; for (int i = 0; i < 4; ++i) { int nx = current.x + dx[i], ny = current.y + dy[i]; if (nx >= 0 && nx < n && ny >= 0 && ny < n && !visited_gen[nx][ny]) neighbors[num_neighbors++] = (Point){nx, ny}; } if (num_neighbors > 0) { int choice = rand() % num_neighbors; Point next = neighbors[choice]; if ((num_passages + 2) <= (GRID_SIZE * GRID_SIZE * 2)) { passages[num_passages++] = current; passages[num_passages++] = next; } visited_gen[next.x][next.y] = true; if (stack_top < (n*n)) stack[stack_top++] = next; } else { stack_top--; } } }

// (BFS pathfinder is unchanged)
void find_shortest_path_bfs() { path_length = 0; Point start = {0, 0}, end = {GRID_SIZE - 1, GRID_SIZE - 1}; Point queue[GRID_SIZE * GRID_SIZE]; int head = 0, tail = 0; Point parent[GRID_SIZE][GRID_SIZE]; bool visited_path[GRID_SIZE][GRID_SIZE]; for(int i=0; i<GRID_SIZE; ++i) for(int j=0; j<GRID_SIZE; ++j) visited_path[i][j] = false; queue[tail++] = start; visited_path[start.x][start.y] = true; parent[start.x][start.y] = (Point){-1, -1}; bool found = false; while(head < tail) { Point current = queue[head++]; if(current.x == end.x && current.y == end.y) { found = true; break; } int dx[] = {0, 0, 1, -1}, dy[] = {1, -1, 0, 0}; for(int i=0; i<4; ++i) { Point next = {current.x + dx[i], current.y + dy[i]}; if(next.x >= 0 && next.x < GRID_SIZE && next.y >= 0 && next.y < GRID_SIZE && !visited_path[next.x][next.y] && !has_wall_c(current, next)) { visited_path[next.x][next.y] = true; parent[next.x][next.y] = current; queue[tail++] = next; } } } if(found) { Point at = end; while(at.x != -1) { path_solution[path_length++] = at; at = parent[at.x][at.y]; } for(int i=0; i < path_length / 2; ++i) { Point temp = path_solution[i]; path_solution[i] = path_solution[path_length - 1 - i]; path_solution[path_length - 1 - i] = temp; } printf("Path found with length: %d\n", path_length); } }

void build_and_spawn_maze() {
    clear_current_maze();
    WbNodeRef root_node = wb_supervisor_node_get_root();
    WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
    wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, "DEF MAZE_GROUP Group {}");
    maze_parent_node_ref = wb_supervisor_node_get_from_def("MAZE_GROUP");
    if (!maze_parent_node_ref) return;
    WbFieldRef maze_children_field = wb_supervisor_node_get_field(maze_parent_node_ref, "children");

    generate_maze_passages_c(GRID_SIZE);
    char vrml_buffer[512];

    // --- BUILD THE MAZE WALLS (MODIFIED TO BE SEAMLESS) ---
    // 1. Draw posts at every intersection
    for (int j = 0; j <= GRID_SIZE; ++j) {
        for (int i = 0; i <= GRID_SIZE; ++i) {
            snprintf(vrml_buffer, sizeof(vrml_buffer), POST_TEMPLATE, MAZE_OFFSET + i * CELL_SIZE, MAZE_OFFSET + j * CELL_SIZE);
            wb_supervisor_field_import_mf_node_from_string(maze_children_field, -1, vrml_buffer);
        }
    }
    // 2. Draw horizontal walls of the correct, shorter length
    for (int j = 0; j <= GRID_SIZE; ++j) {
        for (int i = 0; i < GRID_SIZE; ++i) {
            if (has_wall_c((Point){i, j - 1}, (Point){i, j})) {
                double wx = MAZE_OFFSET + i * CELL_SIZE + CELL_SIZE / 2.0;
                double wy = MAZE_OFFSET + j * CELL_SIZE;
                snprintf(vrml_buffer, sizeof(vrml_buffer), WALL_TEMPLATE, wx, wy, 0.0, MAZE_WALL_LENGTH, MAZE_WALL_LENGTH);
                wb_supervisor_field_import_mf_node_from_string(maze_children_field, -1, vrml_buffer);
            }
        }
    }
    // 3. Draw vertical walls of the correct, shorter length
    for (int i = 0; i <= GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            if ((i == 0 && j == 0) || (i == GRID_SIZE && j == GRID_SIZE - 1)) continue;
            if (has_wall_c((Point){i - 1, j}, (Point){i, j})) {
                double wx = MAZE_OFFSET + i * CELL_SIZE;
                double wy = MAZE_OFFSET + j * CELL_SIZE + CELL_SIZE / 2.0;
                snprintf(vrml_buffer, sizeof(vrml_buffer), WALL_TEMPLATE, wx, wy, 1.5708, MAZE_WALL_LENGTH, MAZE_WALL_LENGTH);
                wb_supervisor_field_import_mf_node_from_string(maze_children_field, -1, vrml_buffer);
            }
        }
    }
    printf(ANSI_COLOR_GREEN "Maze build complete." ANSI_COLOR_RESET "\n");

    find_shortest_path_bfs();

    // Draw the seamless path (this logic is already correct)
    if(path_length > 0) {
        printf("Drawing " ANSI_COLOR_MAGENTA "seamless solution path..." ANSI_COLOR_RESET "\n");
        for(int i = 0; i < path_length - 1; ++i) {
            Point p1 = path_solution[i];
            Point p2 = path_solution[i+1];
            double wx = MAZE_OFFSET + (p1.x + p2.x) / 2.0 * CELL_SIZE + CELL_SIZE / 2.0;
            double wy = MAZE_OFFSET + (p1.y + p2.y) / 2.0 * CELL_SIZE + CELL_SIZE / 2.0;
            double rot = (p1.x != p2.x) ? 0.0 : 1.5708;
            snprintf(vrml_buffer, sizeof(vrml_buffer), PATH_WALL_TEMPLATE, wx, wy, rot);
            wb_supervisor_field_import_mf_node_from_string(maze_children_field, -1, vrml_buffer);
        }
        for(int i = 0; i < path_length; ++i) {
            Point p = path_solution[i];
            double px = MAZE_OFFSET + p.x * CELL_SIZE + CELL_SIZE / 2.0;
            double py = MAZE_OFFSET + p.y * CELL_SIZE + CELL_SIZE / 2.0;
            snprintf(vrml_buffer, sizeof(vrml_buffer), PATH_POST_TEMPLATE, px, py);
            wb_supervisor_field_import_mf_node_from_string(maze_children_field, -1, vrml_buffer);
        }
    }
}

int main(int argc, char **argv) {
  wb_robot_init();
  printf("\n" ANSI_COLOR_CYAN "--- Obstacle Editor Control Panel ---\n");
  printf("Open this URL: " ANSI_COLOR_YELLOW "http://localhost:1234/robot_windows/object_spawner_window/object_spawner_window.html?name=maze_spawner" ANSI_COLOR_RESET "\n");
  printf(ANSI_COLOR_CYAN "-------------------------------------\n\n" ANSI_COLOR_RESET);
  srand(time(NULL));
  initialize_vrml_templates();
  if (!wb_robot_get_supervisor()) { /*...*/ return -1; }
  printf("Maze Supervisor Controller started.\n");
  build_and_spawn_maze();
  while (wb_robot_step(TIME_STEP) != -1) {
    const char *message;
    while ((message = wb_robot_wwi_receive_text())) {
      printf("Received message: " ANSI_COLOR_BLUE "'%s'" ANSI_COLOR_RESET "\n", message);
      if (strcmp(message, "randomize_maze") == 0) {
        printf(ANSI_COLOR_GREEN "Randomize maze command received." ANSI_COLOR_RESET "\n");
        build_and_spawn_maze();
      }
    }
  }
  wb_robot_cleanup();
  return 0;
}