from grid import *
from robot import *
import time
import math
from utils import *


def get_wheel_velocities(robbie, coord, pickup_marker=False, threshold=0.1):
    """
    Helper function to determine the velocities of the robot's left and right wheels.
    Arguments:
        robbie: instance of the robot
        coord (tuple): coordinate to move to (x,y)
        pickup_marker (bool): Only set to 'True' when picking up marker
        threshold (int): Set to expected heading when trying to align robot with marker

    Returns:
        vr, vl: velocities of the robot's left and right wheels
    """

    # Calculate the desired change in position
    controller = PidController()
    dx_world = coord[0] - robbie.x
    dy_world = coord[1] - robbie.y
    dx_robot, dy_robot = rotate_point(dx_world, dy_world, robbie.h)

    # Turn in place
    if not pickup_marker:
        angle = math.atan2(dy_robot, dx_robot)
        if angle < -threshold:
            return -0.02, 0.02
        elif angle > threshold:
            return 0.02, -0.02
    else:
        angle = robbie.h
        if angle < threshold:
            return 0.02, -0.02
        elif angle > threshold:
            return -0.02, 0.02

    robot_pose = np.array([robbie.xyh[0], robbie.xyh[1], robbie.xyh[2]])
    goalpoint = np.array([coord[0], coord[1]])
    linear_v = controller.linear_controller(robot_pose, goalpoint)
    w = controller.angular_controller(robot_pose, goalpoint)
    vl = linear_v - robbie.wheel_dist / 2 * w
    vr = linear_v + robbie.wheel_dist / 2 * w
    return vr, vl


# def phase2_planning(robbie, grid):
#     """
#     This function should move the robot from it's starting position to a marker and then 'pick up' the marker.
#     Arguments:
#         robbie: instance of robot class
#         grid: instance of grid class
#     Returns:
#         robbie: 'updated' instance of robot class
#     Notes:
#         Markers for each grid can be accessed through grid.markers
#         Sample Pseudocode (this is just to give you an idea of how to implement this function. It MAY NOT be a complete solution):
#         1. Move the robot from its current position to a marker on the grid. Use the 'get_wheel_velocities' function to determine the robot's velocities.
#            Note that the 'get_wheel_velocities' function relies on your PIDController implementation in utils.py.
#            You may use the 'rrt' function (see grid.py) when the robot encounters an obstacle.
#         2. When the robot reaches a marker, it must orient itself in the same orientation as the marker so as to 'pick it up'.
#            For example if the marker's orientation is 'R', once the robot gets to the marker, it should turn in place till it's heading is 0 degrees.
#            The 'get_wheel_velocities' function may be used to accomplish this. Note that you must set the 'pickup_marker' variable to 'True' when calling it.
#            Threshold would also need to be set to the appropriate heading for each marker.
#            The expected heading for each of the markers can be accessed by calling the 'parse_marker_info' function in grid.py
#         3. You may keep track of rrt path (if using rrt) by storing in the 'path' function member and current marker by storing in the 'curr_marker' function member
#            in the robot class (check robot.py).

#     Alert:
#         In this part, the robot is expected to 'pick up' all markers by going the desired locations.
#         You may call 'grid.markers' to get the markers' coordinates.
#         However, modifying elements in 'robot.markers_found_or_picked' is prohibited.
#     """
#     ###TODO: Student code here #

#     for marker in grid.markers:
#         marker_x, marker_y, marker_orientation = parse_marker_info(*marker)

#         path_to_marker = grid.rrt(robbie.xy, (marker_x, marker_y))

#         # Navigate to marker
#         for node in path_to_marker:
#             robbie.next_coord = node.xy
#             vr, vl = get_wheel_velocities(robbie, robbie.next_coord)

#             # Check for immediate collision before updating velocities
#             if not grid.is_collision_with_obstacles((robbie.x, robbie.y), robbie.next_coord):
#                 robbie.vr = vr
#                 robbie.vl = vl
#             else:
#                 # Find a new path if there's an immediate obstacle
#                 new_path = grid.rrt((robbie.x, robbie.y), robbie.next_coord)
#                 if new_path:
#                     robbie.next_coord = new_path[1].xy if len(new_path) > 1 else new_path[0].xy
#                     vr, vl = get_wheel_velocities(robbie, robbie.next_coord)
#                     robbie.vr = vr
#                     robbie.vl = vl
#                 else:
#                     print("Unable to find a new path to the marker...")
#                     return robbie

#         robbie.curr_marker = marker  # Simulate marker pick up

#     return robbie


# def phase2_planning(robbie, grid):
#     for marker in grid.markers:
#         marker_x, marker_y, marker_orientation = parse_marker_info(*marker)

#         # Plan a path to the marker using RRT
#         path_to_marker = grid.rrt(robbie.xy, (marker_x, marker_y))

#         if not path_to_marker:
#             print(f"Unable to find a path to marker at ({marker_x}, {marker_y})")
#             continue

#         # Navigate to marker
#         for node in path_to_marker:
#             robbie.next_coord = node.xy
#             vr, vl = get_wheel_velocities(robbie, robbie.next_coord)
#             robbie.vr = vr
#             robbie.vl = vl

#             if grid.is_collision_with_obstacles((robbie.x, robbie.y), robbie.next_coord):
#                 # Handle obstacle collision
#                 new_path = grid.rrt((robbie.x, robbie.y), (marker_x, marker_y))
#                 if new_path:
#                     for new_node in new_path[1:]:  # Skip the first node as it's the current position
#                         robbie.next_coord = new_node.xy
#                         vr, vl = get_wheel_velocities(robbie, robbie.next_coord)
#                         robbie.vr = vr
#                         robbie.vl = vl
#                         robbie.move_diff_drive(grid, vl, vr, robbie.TIMESTEP)
#                     break
#                 else:
#                     print("Unable to find a new path to the marker due to an obstacle.")
#                     return robbie

#             robbie.move_diff_drive(grid, vl, vr, robbie.TIMESTEP)

#         # Align with marker
#         desired_heading = get_heading_for_marker(marker_orientation)
#         if desired_heading is not None:
#             while abs(robbie.h - desired_heading) > 0.1:  # Adjust threshold, was 10
#                 vr, vl = get_wheel_velocities(robbie, robbie.xy, pickup_marker=True, threshold=0.1)  # Same here
#                 robbie.move_diff_drive(grid, vl, vr, robbie.TIMESTEP)

#         # Simulate marker pick up
#         robbie.curr_marker = marker

#     return robbie


# def get_heading_for_marker(marker_orientation):
#     orientations = {"U": 270, "D": 90, "L": 180, "R": 0}
#     return orientations.get(marker_orientation, None)


# def phase2_planning(robbie, grid):
#     DIR_MAP = {"U": 270, "D": 90, "L": 180, "R": 0}
#     REV_DIR_MAP = {270: "U", 90: "D", 180: "L", 0: "R"}
#     ROBOT_COORD_THRESHOLD = 0.5

#     if not hasattr(robbie, "markers_remaining"):
#         robbie.markers_remaining = [(x[0] + 0.5, x[1] + 0.5, DIR_MAP[x[2]]) for x in grid.markers]

#     if robbie.next_coord is None or robbie.curr_marker is None or grid_distance(robbie.x, robbie.y, robbie.next_coord[0], robbie.next_coord[1]) < ROBOT_COORD_THRESHOLD:
#         if robbie.curr_marker is None or len(robbie.path) <= 1:
#             if robbie.curr_marker and (int(robbie.curr_marker[0]), int(robbie.curr_marker[1]), REV_DIR_MAP[robbie.curr_marker[2]]) in robbie.read_marker_around(grid):
#                 robbie.markers_remaining.remove(robbie.curr_marker)
#             robbie.curr_marker = min(robbie.markers_remaining, key=lambda x: grid_distance(robbie.x, robbie.y, x[0], x[1]))
#             robbie.next_coord = (robbie.curr_marker[0], robbie.curr_marker[1])
#             robbie.path = []
#         else:
#             robbie.next_coord = robbie.path.pop(0)

#     if grid.is_collision_with_obstacles(robbie.xy, robbie.next_coord):
#         robbie.path = grid.rrt(robbie.xy, robbie.next_coord)
#         if len(robbie.path) <= 1:
#             raise Exception("No path found")
#         else:
#             robbie.path.pop(0)
#             robbie.path.append((robbie.curr_marker[0], robbie.curr_marker[1]))
#             robbie.next_coord = robbie.path.pop(0)

#     robbie.vr, robbie.vl = get_wheel_velocities(robbie, robbie.next_coord)
#     return robbie


def phase2_planning(robbie, grid):
    DIR_MAP = {"U": 270, "D": 90, "L": 180, "R": 0}
    REV_DIR_MAP = {270: "U", 90: "D", 180: "L", 0: "R"}
    ROBOT_COORD_THRESHOLD = 0.2

    if not hasattr(robbie, "markers_remaining"):
        robbie.markers_remaining = [(x[0] + 0.5, x[1] + 0.5, DIR_MAP[x[2]]) for x in grid.markers]

    need_new_target = robbie.next_coord is None or robbie.curr_marker is None or grid_distance(robbie.x, robbie.y, robbie.next_coord[0], robbie.next_coord[1]) < ROBOT_COORD_THRESHOLD

    if need_new_target:
        if robbie.curr_marker is None or len(robbie.path) <= 1:
            if robbie.curr_marker and (int(robbie.curr_marker[0]), int(robbie.curr_marker[1]), REV_DIR_MAP[robbie.curr_marker[2]]) in robbie.markers_found_or_picked:
                robbie.markers_remaining.remove(robbie.curr_marker)
            robbie.curr_marker = min(robbie.markers_remaining, key=lambda x: grid_distance(robbie.x, robbie.y, x[0], x[1]))
            robbie.next_coord = robbie.curr_marker[:2]
            robbie.path = [Node(robbie.next_coord)]
        else:
            robbie.next_coord = robbie.path.pop(0).xy
    else:
        robbie.vr, robbie.vl = get_wheel_velocities(robbie, robbie.next_coord)
        return robbie

    if grid.is_collision_with_obstacles(robbie.xy, robbie.next_coord):
        robbie.path = grid.rrt(robbie.xy, robbie.next_coord)
        if not robbie.path:
            raise Exception("No path found")
        robbie.path.pop(0)
        if robbie.path[-1].xy != robbie.curr_marker[:2]:
            robbie.path.append(Node(robbie.curr_marker[:2]))
        robbie.next_coord = robbie.path.pop(0).xy

    robbie.vr, robbie.vl = get_wheel_velocities(robbie, robbie.next_coord)
    return robbie
