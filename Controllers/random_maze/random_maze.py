#!/usr/bin/env python3
"""
maze_supervisor_final.py
────────────────────────
Generates a 10x10 maze for an XY-plane world with perfect corners and two fixed openings.
This version uses a post-and-wall system for seamless, visually correct corners.

- Entrance: Fixed at the bottom-left.
- Exit:     Fixed at the top-right.
- Press 'R' to regenerate a new maze.
"""
from controller import Supervisor, Keyboard
import random

# --- GEOMETRY & LAYOUT (FOR XY-PLANE WORLD) ---
GRID   = 10          # 10x10 cells
CELL   = 0.166       # Size of one cell in meters
THK    = 0.05        # Wall thickness
HGT    = 0.05        # Wall height (along the Z-axis)
Z0     = HGT / 2     # Z-position to sit flush on the XY floor
OFFSET = -(GRID * CELL) / 2 # Offset to center the maze at world (0,0)

# --- VRML TEMPLATES ---
# A small square post for corners and junctions
POST_TPL = (
    f'Solid {{ translation {{x}} {{y}} {Z0} '
    f'children [ Shape {{ geometry Box {{ size {THK} {THK} {HGT} }} '
    f'appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 }} }} ] '
    f'boundingObject Box {{ size {THK} {THK} {HGT} }} }}'
)
# A wall segment to connect two posts
WALL_TPL = (
    f'Solid {{ translation {{x}} {{y}} {Z0} '
    f'rotation 0 0 1 {{rot}} ' # Rotate around Z-axis
    f'children [ Shape {{ geometry Box {{ size {CELL} {THK} {HGT} }} '
    f'appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 }} }} ] '
    f'boundingObject Box {{ size {CELL} {THK} {HGT} }} }}'
)

# --- SUPERVISOR SETUP ---
sup = Supervisor()
kb  = sup.getKeyboard(); kb.enable(int(sup.getBasicTimeStep()))
root_field = sup.getRoot().getField('children')
maze_nodes = []

def spawn(vrml: str):
    """Spawns a node from a VRML string and stores its handle."""
    node = root_field.importMFNodeFromString(-1, vrml)
    if node:
        maze_nodes.append(node)

def clear_maze():
    """Removes all nodes that are part of the current maze."""
    for node in maze_nodes:
        if node and node.exists():
            node.remove()
    maze_nodes.clear()

# --- MAZE GENERATION LOGIC ---
def generate_passages(n=GRID):
    passages = set()
    visited = {(0, 0)}
    stack = [(0, 0)]
    # N/S is along Y-axis, E/W is along X-axis
    dirs = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}
    while stack:
        cx, cy = stack[-1]
        neighbors = [
            (d, (cx + dx, cy + dy))
            for d, (dx, dy) in dirs.items()
            if 0 <= cx + dx < n and 0 <= cy + dy < n and (cx + dx, cy + dy) not in visited
        ]
        if neighbors:
            d, (nx, ny) = random.choice(neighbors)
            passages.add(((cx, cy), (nx, ny)))
            visited.add((nx, ny))
            stack.append((nx, ny))
        else:
            stack.pop()
    return passages

# --- MAZE CONSTRUCTION ---
def build_maze():
    """Builds and spawns the maze walls, posts, and openings."""
    clear_maze()
    passages = generate_passages()

    def has_wall(p1, p2):
        return (p1, p2) not in passages and (p2, p1) not in passages

    # 1. Place a post at EVERY grid intersection for a solid frame.
    for j in range(GRID + 1):
        for i in range(GRID + 1):
            px = OFFSET + i * CELL
            py = OFFSET + j * CELL
            spawn(POST_TPL.replace('{x}', f'{px:.4f}').replace('{y}', f'{py:.4f}'))

    # 2. Place HORIZONTAL walls between posts.
    for j in range(GRID + 1):
        for i in range(GRID):
            if has_wall((i, j - 1), (i, j)):
                wx = OFFSET + i * CELL + CELL / 2
                wy = OFFSET + j * CELL
                spawn(WALL_TPL.replace('{x}', f'{wx:.4f}').replace('{y}', f'{wy:.4f}')
                              .replace('{rot}', '0'))

    # 3. Place VERTICAL walls between posts, SKIPPING the openings.
    for i in range(GRID + 1):
        for j in range(GRID):
            # --- OPENING LOGIC ---
            # Entrance at bottom-left: Don't build wall at grid line i=0, cell j=0
            if i == 0 and j == 0:
                continue
            # Exit at top-right: Don't build wall at grid line i=GRID, cell j=GRID-1
            if i == GRID and j == GRID - 1:
                continue
            
            if has_wall((i - 1, j), (i, j)):
                wx = OFFSET + i * CELL
                wy = OFFSET + j * CELL + CELL / 2
                spawn(WALL_TPL.replace('{x}', f'{wx:.4f}').replace('{y}', f'{wy:.4f}')
                              .replace('{rot}', '1.5708'))

# --- MAIN LOOP ---
build_maze()
print("Maze built with seamless corners. Press 'R' to generate a new one.")

timestep = int(sup.getBasicTimeStep())
while sup.step(timestep) != -1:
    key = kb.getKey()
    if key == ord('R'):
        print("Regenerating maze...")
        build_maze()