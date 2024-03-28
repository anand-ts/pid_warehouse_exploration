from robot import Robot
from utils import *
import math


def get_wheel_velocities(robbie, coord):
    """
    Helper function to determine the velocities of the robot's left and right wheels.
    Arguments:
        robbie: instance of the robot
        coord (tuple): coordinate to move to (x,y)

    Returns:
        vr, vl: velocities of the robot's left and right wheels
    """

    # Calculate the desired change in position
    dx_world = coord[0] - robbie.x
    dy_world = coord[1] - robbie.y
    dx_robot, dy_robot = rotate_point(dx_world, dy_world, robbie.h)
    dist_to_coord = math.sqrt(dx_robot**2 + dy_robot**2)

    # Turn in place first
    angle = math.atan2(dy_robot, dx_robot)
    threshold = 1  # changed from 0.1
    if angle < -threshold:
        return -0.025, 0.025  # changed from 0.01
    elif angle > threshold:
        return 0.025, -0.025  # changed from 0.01

    # Using desired linear velocity, set left and right wheel velocity
    linear_v = 0.1 * dist_to_coord  # changed from 0.05
    w = 0.3 * math.atan2(dy_robot, dx_robot)
    vl = linear_v - robbie.wheel_dist / 2 * w
    vr = linear_v + robbie.wheel_dist / 2 * w
    return vr, vl


def frontier_planning(robbie, grid):
    """
    OPTIONAL: Function for defining frontier planning.

    Arguments:
        robbie: instance of the robot
        grid: instance of the grid

    Returns:
        robbie: 'updated' instance of the robot
        OPTIONAL: robbie.next_coord: new destination coordinate

    Notes:
        The lecture notes should provide you with an ample description of frontier planning.
        You will also find many of the functions declared in 'grid.py' and 'utils.py' useful.

    """
    ###TODO: Student Code here ###

    explored_cells = robbie.explored_cells
    frontiers = []

    # Identify frontiers: edges between explored and unexplored areas
    for cell in explored_cells:
        x, y = cell
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                new_x, new_y = x + dx, y + dy
                if (new_x, new_y) not in explored_cells and grid.is_free(new_x, new_y):
                    frontiers.append((new_x, new_y))

    # Choose the nearest frontier
    nearest_frontier = None
    min_distance = float("inf")
    for frontier in frontiers:
        distance = grid_distance(robbie.x, robbie.y, frontier[0], frontier[1])
        if distance < min_distance:
            nearest_frontier = frontier
            min_distance = distance

    # If a frontier is found, set it as the next target
    if nearest_frontier:
        robbie.next_coord = nearest_frontier
    else:
        if all_areas_explored_or_unreachable(robbie, grid):
            print("Exploration complete or all reachable areas have been explored.")
            robbie.next_coord = get_return_coordinate(robbie)
        else:
            print("Attempting to revisit unexplored areas.")
            robbie.next_coord = find_alternate_path_to_unexplored_area(robbie, grid)

    return robbie


def all_areas_explored_or_unreachable(robbie, grid):
    total_free_cells = len(grid.empty)
    return len(robbie.explored_cells) == total_free_cells


def get_return_coordinate(robbie):
    return robbie.start


def find_alternate_path_to_unexplored_area(robbie, grid):
    unexplored_cells = [(x, y) for x in range(grid.width) for y in range(grid.height) if grid.is_free(x, y) and (x, y) not in robbie.explored_cells]
    nearest_unexplored = None
    min_distance = float("inf")

    for cell in unexplored_cells:
        distance = grid_distance(robbie.x, robbie.y, cell[0], cell[1])
        if distance < min_distance:
            nearest_unexplored = cell
            min_distance = distance

    return nearest_unexplored


def exploration_state_machine(robbie, grid):
    """
    Use frontier planning, or another exploration algorithm, to explore the grid.

    Arguments:
        robbie: instance of the robot
        grid: instance of the grid

    Returns:
        robbie: 'updated' instance of the robot

    Notes:
        Robot is considered as Point object located at the center of the traingle.
        You may use the 'rrt' function (see grid.py) to find a new path whenever the robot encounters an obstacle.
        You can use 'grid.is_collision_with_obstacles()' to check if the robot encounters an obstacle.
        Please note that the use of rrt slows down your code, so it should be used sparingly.
        The 'get_wheel_velocities' functions is useful in setting the robot's velocities.
        You will also find many of the functions declared in 'grid.py' and 'utils.py' useful.
        Feel free to create other helper functions (in this file) as necessary.

    Alert:
        In this part, the task is to let the robot find all markers by exploring the map,
        which means using 'grid.markers' will lead  cause zero point on GraderScope.

    """
    ###TODO: Student Code here ###

    robbie.get_cells_in_fov(grid)

    robbie = frontier_planning(robbie, grid)

    if robbie.next_coord:
        vr, vl = get_wheel_velocities(robbie, robbie.next_coord)

        robbie.vr = vr
        robbie.vl = vl

        # Check for obstacles along the path
        if grid.is_collision_with_obstacles((robbie.x, robbie.y), robbie.next_coord):
            # Immediate stop to prevent collision
            robbie.vr, robbie.vl = 0, 0

            # Find a new path using RRT if there is an obstacle
            path = grid.rrt((robbie.x, robbie.y), robbie.next_coord)
            if path:
                robbie.next_coord = path[1].xy if len(path) > 1 else path[0].xy
                vr, vl = get_wheel_velocities(robbie, robbie.next_coord)
                robbie.vr = vr
                robbie.vl = vl
            else:
                print("Unable to find a new path. Stopping exploration.")
                return robbie

        try:
            robbie.move_diff_drive(grid, vl, vr, robbie.TIMESTEP)
        except Exception as e:
            print(f"Encountered an error while moving: {e}")
            robbie.vr, robbie.vl = 0, 0

    return robbie


# def perform_360_scan(robbie, grid):
#     original_heading = robbie.h
#     current_heading = original_heading
#     while True:
#         # Set wheel velocities for spinning
#         robbie.vr, robbie.vl = 0.05, -0.05
#         robbie.move_diff_drive(grid, robbie.vl, robbie.vr, robbie.TIMESTEP)

#         # Update current heading
#         current_heading = (current_heading + 360 + 0.02 * robbie.TIMESTEP) % 360

#         # Break the loop once a full 360-degree turn is completed
#         if abs(current_heading - original_heading) < 1:  # 1 degree tolerance
#             break

#     # Reset wheel velocities after scan
#     robbie.vr, robbie.vl = 0, 0
