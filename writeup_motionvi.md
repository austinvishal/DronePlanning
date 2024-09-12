## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

--- Vish


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code
MotionPlanning is called in __main__. 
Several callback functions are defined here. 
The main event cycle is as follows. 
MANUAL->ARMING->PLANNING->TAKEOFF->WAYPOINT->LANDING->DISARMING->MANUAL
After ARMING event, plan_path() is called to calculate waypoints for flying from start to goal.
In plan_path(), first current event is set to PLANNING. 
A 2.5D grid map is loaded by calling create_grid(). 
The start is set to the center of the grid map and the goal is set to about 14 meters Northern-east direction from the center. 
Then a_star is called to calculate the waypoints.
In a_star(), all possible paths are calculated and the one with lowest cost will be returned as target waypoints for flying.
When received waypoints, event goes from PLANNING to TAKEOFF then to WAYPOINT, flying from the first waypoint, one by another, to the last waypoint.
After reaching the last waypoint, the event goes to LANDING and the drone begins to land. When the altitude is close to the ground, the event finally changes to DISARMING->MANUAL.

1. `motion_planning.py` implements a drone with state transition and path planning.
The main event cycle is as follows. MANUAL->ARMING->PLANNING->TAKEOFF->WAYPOINT->LANDING->DISARMING->MANUAL

2. `planning_utils.py` are planning functions that discretize the map data, perform grid search and truncate waypoints.


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
The header of the file contains latitude and longitude information of the map center and is extraced using `numpy.genfromtxt`.
```python
with open('colliders.csv') as f:
            first_line = f.readline()
        items = first_line.split(',')

        lat0 = float(items[0].split()[1])
        lon0 = float(items[1].split()[1])

        # set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)
```

#### 2. Set your current local position
When the global home is defined, current local position is computed by calling 
```python
local_pos = global_to_local(self.global_position, self.global_home). 

```
local_pos is the distance that is relative to global home.


#### 3. Set grid start position from local position
The start location is defined as the drone`s current local location.If grid search is used, the start position has to be offset from the local position. 

```python
grid_start = (int(local_pos[0])-north_offset, int(local_pos[1])-east_offset)

```
#### 4. Set grid goal position from geodetic coords
The goal location is defined in geodetic coords format. The initial values are defined in goal_lon and goal_lat. It is then converted to grid location from corresponding local position subtracting north/east offset.
```python
goal_lon = -122.398470
        goal_lat = 37.793445
        local_goal = global_to_local((goal_lon, goal_lat, 0), self.global_home)
        grid_start = (int(local_pos[0])-north_offset, int(local_pos[1])-east_offset)
        grid_goal = (int(local_goal[0]-north_offset), int(local_goal[1]-east_offset))

```
#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Diagnol motion included. The cost of diagonal motions is set to root 2 as default. The validity of each direction is also tested here similar to the original four directions.
```python
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    # diagonal actions and their costs
    NORTHWEST = (-1, -1, np.sqrt(2))
    NORTHEAST = (-1, 1, np.sqrt(2))
    SOUTHWEST = (1, -1, np.sqrt(2))
    SOUTHEAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node
    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
    if (x - 1 < 0 or y - 1 < 0 )or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTHWEST)
    if (x - 1 < 0 or  y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTHEAST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTHEAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTHWEST)
    return valid_actions

       
def a_star_graph(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""

    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node)

    path = []
    path_cost = 0
    if found:

        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])

    return path[::-1], path_cost
```
#### 6. Cull waypoints 
After the initial waypoints are computed by a_star, 
they are pruned in function prune_path defined in MotionPlanning class. 
in prune_path, collinearity_check is done to eliminate redundant waypoints. 
Also, since diagonal motions became available, there might be motions travelling two sides of a right triangle 
which are checked using right triangle check

```python
def collinearity_check(self, p1, p2, p3, epsilon=1e-6):
        m = np.concatenate((p1, p2, p3), axis=0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    def right_triangle_check(self, p1, p2, p3, epsilon=1e-6):
        distance1 = np.linalg.norm(p1-p2)
        distance2 = np.linalg.norm(p2-p3)
        distance3 = np.linalg.norm(p1-p3)
        case1 =  distance1**2 + distance2**2 - distance3**2 < epsilon
        case2 = distance1 == distance2
        return case1 and case2

    def prune_path(self, path):
        pruned_path = [p for p in path]
        i = 0
        # First remove collinear waypoints and right triangle waypoints
        while i < len(pruned_path) - 2:
            p1 = self.point(pruned_path[i])
            p2 = self.point(pruned_path[i+1])
            p3 = self.point(pruned_path[i+2])
            if self.collinearity_check(p1, p2, p3) or self.right_triangle_check(p1, p2, p3):
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
        i = 0
        # Second, remove the redundant waypoints
        while i < len(pruned_path) - 2:
            p1 = self.point(pruned_path[i])
            p2 = self.point(pruned_path[i+1])
            p3 = self.point(pruned_path[i+2])
            if self.collinearity_check(p1, p2, p3):
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
        return pruned_path
```


### Execute the flight
#### 1. Does it work?
It does, gives pruned path. sometimes it times out due to network error!

