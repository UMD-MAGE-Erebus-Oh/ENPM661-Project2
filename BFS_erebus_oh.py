"""Breadth-First Seearch - Project 2

Erebus Oh

y
^
|
(0, 0) ----> x

"""

import matplotlib.pyplot as plt
import matplotlib.image
import numpy as np
from tqdm import tqdm

# We use the shapely library to help us create the obstacles and collision code
from shapely.geometry import Point, LinearRing
from shapely.geometry.polygon import Polygon
import shapely
from shapely import affinity

from typing import List
import math

CLEARANCE = 2

BLUE = (0, 0, 255)
RED = (255, 0, 0)

# Step 1: Define Actions in Methematical Format
# 


# Step 2: Find the Mathematical Representation of Free Space
# We will be using the shapely library for drawing obstacle shapes
# and checking for collisions/clearance

class Obstacle():

    def __init__(
        self, 
        x_coords: List[int],
        y_coords: List[int], 
        xoffset: int = 0, 
        yoffset: int = 0, 
        clearance: int = 2
    ):
        # https://shapely.readthedocs.io/en/2.0.6/reference/shapely.LinearRing.html#shapely.LinearRing
        # https://shapely.readthedocs.io/en/2.0.6/reference/shapely.Polygon.html#shapely.Polygon
        self.boundary = LinearRing(np.vstack((x_coords, y_coords)).T)
        self.poly = Polygon(self.boundary)
        self.xoffset = xoffset
        self.yoffset = yoffset
        self.clearance = clearance
        # https://shapely.readthedocs.io/en/2.0.6/manual.html#shapely.affinity.translate
        self.poly = affinity.translate(self.poly,xoff=self.xoffset,yoff=self.yoffset)
        self.boundary = affinity.translate(self.boundary,xoff=self.xoffset,yoff=self.yoffset)

    def collides(self, coords) -> bool:
        p = Point(coords)
        
        # whether point is inside boundary (boundary exclusive)
        inside_obstacle = self.poly.contains(p)
        # https://shapely.readthedocs.io/en/2.0.6/reference/shapely.dwithin.html
        # whether point is +/- clearance from boundary
        within_clearance = self.boundary.dwithin(p, self.clearance)

        return inside_obstacle or within_clearance

    def draw(self, ax, color: str = 'blue'):
        ax.fill(*self.boundary.xy, facecolor=color)

# Obstacles

# Straight Letters/Numbers
class LetterE(Obstacle):
    x = [10, 10, 23, 23, 15, 15, 23, 23, 15, 15, 23, 23]
    y = [10, 35, 35, 30, 30, 25, 25, 20, 20, 15, 15, 10]

    def __init__(self):
        super().__init__(
            x_coords = self.x,
            y_coords = self.y,
            clearance = CLEARANCE
        )

class LetterN(Obstacle):
    x = [30, 30, 35, 43, 43, 48, 48, 43, 35, 35]
    y = [10, 35, 35, 20, 35, 35, 10, 10, 22, 10]

    def __init__(self):
        super().__init__(
            x_coords = self.x,
            y_coords = self.y,
            clearance = CLEARANCE
        )

class LetterM(Obstacle):
    x = [70, 70, 75, 80, 83, 88, 93, 93, 88, 88, 85, 78, 75, 75]
    y = [10, 35, 35, 15, 15, 35, 35, 10, 10, 25, 10, 10, 25, 10]

    def __init__(self):
        super().__init__(
            x_coords = self.x,
            y_coords = self.y,
            clearance = CLEARANCE
        )

class Number1(Obstacle):
    x = [150, 150, 155, 155]
    y = [10, 38, 38, 10]

    def __init__(self):
        super().__init__(
            x_coords = self.x,
            y_coords = self.y,
            clearance = CLEARANCE
        )

# Curved Letters/Numbers

def generate_arc(
    x_center, 
    y_center, 
    radius,
    start_theta,
    stop_theta,
    resolution = 1000
):
    # https://stackoverflow.com/questions/30762329/how-to-create-polygons-with-arcs-in-shapely-or-a-better-library
    
    thetas = np.radians(np.linspace(start_theta, stop_theta, resolution))
    x = x_center + radius * np.cos(thetas)
    y = y_center + radius * np.sin(thetas)

    return x, y

class LetterP(Obstacle):

    x1 = [55, 55, 61]
    y1 = [10, 35, 35]

    x_center = 61
    y_center = 29
    radius = 6
    start = 90
    stop = -90

    x2 = [61, 61]
    y2 = [23, 10]

    def __init__(self):

        cx, cy = generate_arc(
            x_center=self.x_center, 
            y_center=self.y_center, 
            radius=self.radius, 
            start_theta=self.start, 
            stop_theta=self.stop
        )

        x_coords = np.concatenate((self.x1, cx, self.x2))
        y_coords = np.concatenate((self.y1, cy, self.y2))

        super().__init__(
            x_coords=x_coords,
            y_coords=y_coords,
            clearance = CLEARANCE
        )

class Number6(Obstacle):

    x_center = 109
    y_center = 19
    radius = 9
    # 63.61 math.acos(4/9)
    start = 116.39
    stop = -180

    x1 = [100, 100, 105, 105]
    y_int = math.sqrt((radius ** 2) - ((105 - 109) ** 2)) + 19
    y1 = [19, 40, 40, y_int]

    def __init__(self, x_offset: int = 0, y_offset: int = 0):

        cx, cy = generate_arc(
            x_center=self.x_center, 
            y_center=self.y_center, 
            radius=self.radius, 
            start_theta=self.start, 
            stop_theta=self.stop
        )

        x_coords = np.concatenate((self.x1, cx))
        y_coords = np.concatenate((self.y1, cy))

        super().__init__(
            x_coords=x_coords,
            y_coords=y_coords,
            xoffset=x_offset,
            yoffset=y_offset,
            clearance = CLEARANCE
        )

class Project2Map():

    height = 50
    width = 180

    map_filename = 'map.png'

    def __init__(self):
        self.obstacles = [
            LetterE(),
            LetterN(),
            LetterP(),
            LetterM(),
            Number6(),
            Number6(x_offset=26, y_offset=0),
            Number1(),
        ]
        # draw map space into image
        self.draw_map(show_fig=True, save = True)

        # load obstacle spaceback in
        # note: (0,0) upper right
        self._map_image = plt.imread('map.png')

    def map(self, x: int, y: int):
        # returns False if obstacle, else True
        # assumes (0,0) bottom left
        # print(f"indexing: {-(y+1) + self.height}, {x}")
        pixel = self._map_image[-(y+1) + self.height, x][:-1].sum()
        return pixel == 0
    
    
    def draw_map(self, show_fig = False, save: bool = True):
        if show_fig:
            fig = plt.figure(figsize=(21, 6))
            ax = fig.add_subplot(111)
            ax.set_xlim(0, 180)
            ax.set_ylim(0, 50)
            # draw obstacles in blue
            for o in self.obstacles:
                o.draw(ax, color = 'blue')

        map_colors = np.zeros((self.height, self.width, 3))
        
        for i in tqdm(range(self.width), desc="Generating Map"):
            for j in range(self.height):
                # free space is green
                if show_fig:
                    ax.plot(i, j, marker='.', color='green')
                for o in self.obstacles:
                    if o.collides([i, j]):
                        # obstacle or clearance is red
                        if show_fig:
                            ax.plot(i, j, marker='.', color='red')
                        map_colors[-(j + 1) + self.height, i] = RED
                        continue
        if save:
            matplotlib.image.imsave(self.map_filename, map_colors/255.0)
            print(f"Saved map as {self.map_filename}")
        
        if show_fig:
            return fig, ax

# Step 3: Generate the Graph and Check for Goal Node in each Iteration
# aka perform BFS