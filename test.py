
from BFS_erebus_oh import Project2Solver, Visualizer

def main():
    # solver = Project2Solver(start = (5, 10), stop = (50,5))
    solver = Project2Solver(start = (120, 30), stop = (27,18))
    v = Visualizer(solver)
    v.animate()

if __name__ == "__main__":
    main()