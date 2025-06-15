/*
 * Controller for a Webots Supervisor robot that spawns objects and mazes
 * based on messages from an HTML robot window.
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

// --- NEW: ANSI COLOR CODE DEFINITIONS ---
#define ANSI_COLOR_RESET   "\x1b[0m"
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
// --- END NEW BLOCK ---

// --- MAZE GEOMETRY & LAYOUT ---
#define GRID_SIZE 10
#define CELL_SIZE 0.20
#define WALL_THICKNESS 0.05
#define WALL_HEIGHT 0.075
#define WALL_Z_POS (WALL_HEIGHT / 2.0)
#define MAZE_OFFSET (-(GRID_SIZE * CELL_SIZE) / 2.0)

// --- VRML TEMPLATES ---
char POST_TEMPLATE[256];
char WALL_TEMPLATE[256];

WbNodeRef maze_parent_node_ref = NULL;

typedef struct { int x; int y; } Point;
Point passages[GRID_SIZE * GRID_SIZE * 2];
int num_passages = 0;
bool visited[GRID_SIZE][GRID_SIZE];

// --- FUNCTION DECLARATIONS ---
void initialize_vrml_templates();
void generate_maze_passages_c(int n);
void clear_current_maze();
void build_and_spawn_maze();

// (The maze generation and building functions from here are unchanged)
// ...

void initialize_vrml_templates() {
    snprintf(POST_TEMPLATE, sizeof(POST_TEMPLATE),
             "Solid { translation %%f %%f %f "
             "children [ Shape { geometry Box { size %f %f %f } "
             "appearance PBRAppearance { baseColor 0.1 0.1 0.1 } } ] "
             "boundingObject Box { size %f %f %f } }",
             WALL_Z_POS, WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT,
             WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT);

    snprintf(WALL_TEMPLATE, sizeof(WALL_TEMPLATE),
             "Solid { translation %%f %%f %f "
             "rotation 0 0 1 %%f "
             "children [ Shape { geometry Box { size %f %f %f } "
             "appearance PBRAppearance { baseColor 0.1 0.1 0.1 } } ] "
             "boundingObject Box { size %f %f %f } }",
             WALL_Z_POS, CELL_SIZE, WALL_THICKNESS, WALL_HEIGHT,
             CELL_SIZE, WALL_THICKNESS, WALL_HEIGHT);
}

void clear_current_maze() {
    if (maze_parent_node_ref) {
        // MODIFIED: Added color
        printf(ANSI_COLOR_YELLOW "Clearing existing maze (MAZE_GROUP)..." ANSI_COLOR_RESET "\n");
        wb_supervisor_node_remove(maze_parent_node_ref);
        maze_parent_node_ref = NULL;
    }
}

void generate_maze_passages_c(int n) {
    num_passages = 0;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            visited[i][j] = false;

    Point stack[n * n];
    int stack_top = 0;

    stack[stack_top++] = (Point){0, 0};
    visited[0][0] = true;

    int dx[] = {0, 0, 1, -1};
    int dy[] = {1, -1, 0, 0};

    while (stack_top > 0) {
        Point current = stack[stack_top - 1];
        Point neighbors[4];
        int num_neighbors = 0;

        for (int i = 0; i < 4; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];
            if (nx >= 0 && nx < n && ny >= 0 && ny < n && !visited[nx][ny]) {
                neighbors[num_neighbors++] = (Point){nx, ny};
            }
        }

        if (num_neighbors > 0) {
            int choice = rand() % num_neighbors;
            Point next = neighbors[choice];

            if ((num_passages + 2) <= (GRID_SIZE * GRID_SIZE * 2) ) {
                 passages[num_passages++] = current;
                 passages[num_passages++] = next;
            } else {
                fprintf(stderr, ANSI_COLOR_RED "Error: Too many passages for array.\n" ANSI_COLOR_RESET);
                break; 
            }

            visited[next.x][next.y] = true;
            if(stack_top < (n*n))
                stack[stack_top++] = next;
            else {
                 fprintf(stderr, ANSI_COLOR_RED "Error: Stack overflow in maze generation.\n" ANSI_COLOR_RESET);
                 break;
            }
        } else {
            stack_top--;
        }
    }
}

bool has_wall_c(Point p1, Point p2) {
    for (int i = 0; i < num_passages; i += 2) {
        if ((passages[i].x == p1.x && passages[i].y == p1.y &&
             passages[i+1].x == p2.x && passages[i+1].y == p2.y) ||
            (passages[i].x == p2.x && passages[i].y == p2.y &&
             passages[i+1].x == p1.x && passages[i+1].y == p1.y)) {
            return false;
        }
    }
    return true;
}

void build_and_spawn_maze() {
    clear_current_maze();

    WbNodeRef root_node = wb_supervisor_node_get_root();
    WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
    
    wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, "DEF MAZE_GROUP Group {}");
    maze_parent_node_ref = wb_supervisor_node_get_from_def("MAZE_GROUP");

    if (!maze_parent_node_ref) {
        fprintf(stderr, ANSI_COLOR_RED "ERROR: Could not create MAZE_GROUP node.\n" ANSI_COLOR_RESET);
        return;
    }
    WbFieldRef maze_children_field = wb_supervisor_node_get_field(maze_parent_node_ref, "children");

    generate_maze_passages_c(GRID_SIZE);
    char vrml_buffer[512];

    for (int j = 0; j <= GRID_SIZE; ++j) {
        for (int i = 0; i <= GRID_SIZE; ++i) {
            double px = MAZE_OFFSET + i * CELL_SIZE;
            double py = MAZE_OFFSET + j * CELL_SIZE;
            snprintf(vrml_buffer, sizeof(vrml_buffer), POST_TEMPLATE, px, py);
            wb_supervisor_field_import_mf_node_from_string(maze_children_field, -1, vrml_buffer);
        }
    }

    for (int j = 0; j <= GRID_SIZE; ++j) {
        for (int i = 0; i < GRID_SIZE; ++i) {
            if (has_wall_c((Point){i, j - 1}, (Point){i, j})) {
                double wx = MAZE_OFFSET + i * CELL_SIZE + CELL_SIZE / 2.0;
                double wy = MAZE_OFFSET + j * CELL_SIZE;
                snprintf(vrml_buffer, sizeof(vrml_buffer), WALL_TEMPLATE, wx, wy, 0.0);
                wb_supervisor_field_import_mf_node_from_string(maze_children_field, -1, vrml_buffer);
            }
        }
    }

    for (int i = 0; i <= GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            if (i == 0 && j == 0) continue;
            if (i == GRID_SIZE && j == GRID_SIZE - 1) continue;

            if (has_wall_c((Point){i - 1, j}, (Point){i, j})) {
                double wx = MAZE_OFFSET + i * CELL_SIZE;
                double wy = MAZE_OFFSET + j * CELL_SIZE + CELL_SIZE / 2.0;
                snprintf(vrml_buffer, sizeof(vrml_buffer), WALL_TEMPLATE, wx, wy, 1.5708);
                wb_supervisor_field_import_mf_node_from_string(maze_children_field, -1, vrml_buffer);
            }
        }
    }
    // MODIFIED: Added color
    printf(ANSI_COLOR_GREEN "Maze build complete." ANSI_COLOR_RESET "\n");
}

int main(int argc, char **argv) {
  wb_robot_init();
  
  // MODIFIED: Added colors to this block
  printf("\n" ANSI_COLOR_CYAN "--- Obstacle Editor Control Panel ---" ANSI_COLOR_RESET "\n");
  printf("Open this URL in a web browser to control the simulation:\n");
  printf(ANSI_COLOR_CYAN "-------------------------------------\n\n" ANSI_COLOR_RESET);
  printf(ANSI_COLOR_YELLOW "http://localhost:1234/robot_windows/random_maze_window/random_maze_window.html?name=maze_spawner" ANSI_COLOR_RESET "\n");
  printf(ANSI_COLOR_CYAN "-------------------------------------\n\n" ANSI_COLOR_RESET);
  
  srand(time(NULL));
  initialize_vrml_templates();

  if (!wb_robot_get_supervisor()) {
    fprintf(stderr, ANSI_COLOR_RED "Error: This controller requires the robot to be a Supervisor.\n" ANSI_COLOR_RESET);
    wb_robot_cleanup();
    return -1;
  }
  printf("Maze Supervisor Controller started.\n");

  build_and_spawn_maze();

  while (wb_robot_step(TIME_STEP) != -1) {
    const char *message;
    while ((message = wb_robot_wwi_receive_text())) {
      // MODIFIED: Added colors
      printf("Received message: " ANSI_COLOR_BLUE "'%s'" ANSI_COLOR_RESET "\n", message);
      if (strcmp(message, "randomize_maze") == 0) {
        printf(ANSI_COLOR_GREEN "Randomize maze command received." ANSI_COLOR_RESET "\n");
        build_and_spawn_maze();
      } else if (strncmp(message, "spawn_box", 9) == 0) {
        // This part is unchanged but will benefit from colors in error messages
        double x = 0.0, y = 0.5, z = 0.0;
        int items_scanned = sscanf(message, "spawn_box %lf %lf %lf", &x, &y, &z);
        if (items_scanned != 3) {
            x = (rand() % 400 - 200) / 100.0;
            y = 0.5;
            z = (rand() % 400 - 200) / 100.0;
            printf(ANSI_COLOR_YELLOW "Could not parse coordinates for spawn_box, using random: %.2f %.2f %.2f\n" ANSI_COLOR_RESET, x, y, z);
        } else {
            printf("Parsed coordinates for spawn_box: x=%.2f, y=%.2f, z=%.2f\n", x, y, z);
        }
         WbNodeRef root_node = wb_supervisor_node_get_root();
         WbFieldRef children_field = wb_supervisor_node_get_field(root_node, "children");
         char box_string[256];
         sprintf(box_string, "Transform{translation %f %f %f children[Shape{appearance PBRAppearance{baseColor 1 0 0}geometry Box{size .2 .2 .2}}]}",x,y,z);
         wb_supervisor_field_import_mf_node_from_string(children_field, -1, box_string);
         printf(ANSI_COLOR_GREEN "Spawned a box at %.2f %.2f %.2f\n" ANSI_COLOR_RESET, x,y,z);
      } else {
        fprintf(stderr, ANSI_COLOR_RED "Unknown message: '%s'\n" ANSI_COLOR_RESET, message);
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}