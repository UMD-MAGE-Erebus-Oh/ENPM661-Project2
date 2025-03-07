# Project 2: BFS and Dijkstra for Point Robot

Erebus Oh

ENPM 661, Spring 2025

March 11, 2025

Deliverables
- `README.md`
    - how to run code
    - libraries/dependencies used
- `BFS_erebus_oh.py`
- `dijkstra_erebus_oh.py`
    - inputs: start and goal coords
- Github repo link
- Animation video (.mp4)
    - recording code exploration and optimal path
    - start and goal random

Assumptions
- point robot
- 2mm clearance
- 8 connected (UDLR = 1.0, diagonals = 1.0 for BFS, 1.4 for dijkstra)
- map given

Steps
1. Define actions
    - input: current node
    - outputs: next node
2. Representaion free space
    - 2mm clearance
    - plot space with matplotlib/opencv
3. Generate graph, check for goal node in each iteration
4. Optimal path, backtracking
    - generate path
5. Represent Path/Exploration
    - animation for node exploration and path
