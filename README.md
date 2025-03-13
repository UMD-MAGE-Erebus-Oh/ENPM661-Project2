# Project 2: BFS and Dijkstra for Point Robot

Erebus Oh

ENPM 661, Spring 2025

March 11, 2025

[Github Link](https://github.com/UMD-MAGE-Erebus-Oh/ENPM661-Project2)

## Run This Code
---

### Dependenices
Install the necessary dependencies:
```
pip install -r requirements.txt
```
Or manually install the dependencies:
```
pip install numpy matplotlib tqdm shapely
```

### Ren Solver
To run the solver, just run the `BFS_erebus_oh.py` file:
```bash
python BFS_erebus_oh.py
```
The exploration and path gif will be output as `animation.mp4` if ffmpeg is available (otherwise will be saved as `animation.gif`). As submitted, set to solution for start = (5,10) and goal = (1,1).
- ffmpeg is significantly faster than pillow, to install:
    - Linux: `sudo apt update && sudo apt install ffmpeg`
    - [GeeksForGeeks: How to Install FFmpeg in Linux?](https://www.geeksforgeeks.org/how-to-install-ffmpeg-in-linux/)


## Description

### Deliverables
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

### Assumptions
- point robot
- 2mm clearance
- 8 connected (UDLR = 1.0, diagonals = 1.0 for BFS, 1.4 for dijkstra)
- map given

### Steps
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

### Rubric
- Define robot actions: Define correct actions for BFS and Dijkstra's(optional) algorithm
- Obstacle space creation: Create obstacle space using half planes and semi-algebraic models
- check start and goal point: Check the start and goal point if they fall in obstacle space
- Node exploration and backtracking
    - Implement BFS. The code should have sufficient comments and documentation to explain the approach
    - (Implement Dijkstra's algorithm- optional)
- Visualization
    - Show visualization video demonstrating node exploration and final path generated in form of a video or gif file
- Submissions guidelines followed: Submit the following files
    - README file (.md or .txt)
    - Source Codes (.py)
    - Animation Videos
Runtime
    - The algorithms are able to find a solution(if possible) for any start and goal points in less than 5 minutes