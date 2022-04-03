#!/usr/bin/env python3

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Author: Ryan De Iaco
# Additional Comments: Carlos Wang
# Date: October 29, 2018

import numpy as np
import scipy.spatial
from math import sin, cos, pi, sqrt
from behavioural_planner import manhattan_distance
from copy import copy


class CollisionChecker:
    def __init__(self, circle_offsets, circle_radii, weight):
        self._circle_offsets = circle_offsets
        self._circle_radii   = circle_radii
        self._weight         = weight

    ######################################################
    ######################################################
    # MODULE 7: CHECKING FOR COLLISSIONS
    #   Read over the function comments to familiarize yourself with the
    #   arguments and necessary variables to return. Then follow the TODOs
    #   (top-down) and use the surrounding comments as a guide.
    ######################################################
    ######################################################
    # Takes in a set of paths and obstacles, and returns an array
    # of bools that says whether or not each path is collision free.
    """
    local planner에서 우리는 여러 goal의 possible path들을 계산하였다. 이 path들이 collision free한지 점검할 것이다.
    obstacle의 위치는 주어진다.
    각 path들은 인덱싱되어 paths에 저장되어 있고, 그와 동일한 인덱스를 지닌 boolean 리스트를 작성하여 반환할 것이다.
    Collision free check를 할 때, Circle Model Method을 사용할 것이다.
    """
    def collision_check(self, paths, obstacles):
        """Returns a bool array on whether each path is collision free.

        args:
            paths: A list of paths in the global frame.  
                A path is a list of points of the following format:
                    [x_points, y_points, t_points]:
                        x_points: List of x values (m)
                        y_points: List of y values (m)
                        t_points: List of yaw values (rad)
                    Example of accessing the ith path, jth point's t value:
                        paths[i][2][j]
            obstacles: A list of [x, y] points that represent points along the
                border of obstacles, in the global frame.
                Format: [[x0, y0],
                         [x1, y1],
                         ...,
                         [xn, yn]]
                , where n is the number of obstacle points and units are [m, m]

        returns:
            collision_check_array: A list of boolean values which classifies
                whether the path is collision-free (true), or not (false). The
                ith index in the collision_check_array list corresponds to the
                ith path in the paths list.
        """
        collision_check_array = np.zeros(len(paths), dtype=bool)
        for i in range(len(paths)):# 각 path별로
            collision_free = True
            path           = paths[i]

            # Iterate over the points in the path.
            """
            recall:
            circle model은 curPos/path를 기준으로 offset된 circle들의 set을 형성하고,
            offset은 vehicle의 yaw에 따른 방향으로 진행된다.
            각 circle과 obstacle이 접하는지/안쪽 영역으로 침범하는지를 확인하는 방법이다. 
            여기서는 path별로, 각 parh 내부의 점을 기준으로 원을 형성하고 그에 대한 오프셋을 주어서 각각의 원들을 형성한 뒤, 그들에 대한 값을 계산하는 방식이다.
            """
            for j in range(len(path[0])): # 각 path 내부 점들 별로
                # Compute the circle locations along this point in the path.
                # These circle represent an approximate collision
                # border for the vehicle, which will be used to check
                # for any potential collisions along each path with obstacles.

                # The circle offsets are given by self._circle_offsets.
                # The circle offsets need to placed at each point along the path,
                # with the offset rotated by the yaw of the vehicle.
                # Each path is of the form [[x_values], [y_values],
                # [theta_values]], where each of x_values, y_values, and
                # theta_values are in sequential order.

                # Thus, we need to compute for each point along the path.
                # point_x is given by path[0][j], and point _y is given by
                # path[1][j]. 
                circle_locations = np.zeros((len(self._circle_offsets), 2))

                # TODO: INSERT YOUR CODE BETWEEN THE DASHED LINES
                # --------------------------------------------------------------
                x_offset_list = self._circle_offsets.copy()
                y_offset_list = self._circle_offsets.copy()
                
                for l in range(len(x_offset_list)):
                    x_offset_list[l] = x_offset_list[l]*cos(path[2][j])
                    y_offset_list[l] = y_offset_list[l]*sin(path[2][j])
                circle_locations[:, 0] = path[0][j] + x_offset_list
                circle_locations[:, 1] = path[1][j] + y_offset_list
                # --------------------------------------------------------------

                # Assumes each obstacle is approximated by a collection of
                # points of the form [x, y].
                # Here, we will iterate through the obstacle points, and check
                # if any of the obstacle points lies within any of our circles.
                # If so, then the path will collide with an obstacle and
                # the collision_free flag should be set to false for this flag
                for k in range(len(obstacles)):
                    collision_dists = \
                        scipy.spatial.distance.cdist(obstacles[k], 
                                                     circle_locations)
                    collision_dists = np.subtract(collision_dists, 
                                                  self._circle_radii)
                    collision_free = collision_free and \
                                     not np.any(collision_dists < 0)

                    if not collision_free:
                        break
                if not collision_free:
                    break

            collision_check_array[i] = collision_free

        return collision_check_array

    ######################################################
    ######################################################
    # MODULE 7: SELECTING THE BEST PATH INDEX
    #   Read over the function comments to familiarize yourself with the
    #   arguments and necessary variables to return. Then follow the TODOs
    #   (top-down) and use the surrounding comments as a guide.
    ######################################################
    ######################################################
    # Selects the best path in the path set, according to how closely
    # it follows the lane centerline, and how far away it is from other
    # paths that are in collision. 
    # Disqualifies paths that collide with obstacles from the selection
    # process.
    # collision_check_array contains True at index i if paths[i] is
    # collision-free, otherwise it contains False.
    """
    생성한 path들 중 현재 Collision free인 것들만 우선 필터링되었다. 이제 단 하나의 최종 optimal한 path를 찾아야 한다.
    optimal한 것을 책정하는 데에는 다양한 기준이 있지만, 여기서는 ctrline에 가까울수록, 다른 collision하는 path들로부터 멀수록 '좋다'고 판단하였다.
    """
    def select_best_path_index(self, paths, collision_check_array, goal_state):
        """Returns the path index which is best suited for the vehicle to
        traverse.

        Selects a path index which is closest to the center line as well as far
        away from collision paths.

        args:
            paths: A list of paths in the global frame.  
                A path is a list of points of the following format:
                    [x_points, y_points, t_points]:
                        x_points: List of x values (m)
                        y_points: List of y values (m)
                        t_points: List of yaw values (rad)
                    Example of accessing the ith path, jth point's t value:
                        paths[i][2][j]
            collision_check_array: A list of boolean values which classifies
                whether the path is collision-free (true), or not (false). The
                ith index in the collision_check_array list corresponds to the
                ith path in the paths list.
            goal_state: Goal state for the vehicle to reach (centerline goal).
                format: [x_goal, y_goal, v_goal], unit: [m, m, m/s]
        useful variables:
            self._weight: Weight that is multiplied to the best index score.
        returns:
            best_index: The path index which is best suited for the vehicle to
                navigate with.
        """
        best_index = None
        best_score = float('Inf')
        for i in range(len(paths)):
            # Handle the case of collision-free paths.
            if collision_check_array[i]:
                # Compute the "distance from centerline" score.
                # The centerline goal is given by goal_state.
                # The exact choice of objective function is up to you.
                # A lower score implies a more suitable path.
                # TODO: INSERT YOUR CODE BETWEEN THE DASHED LINES
                # --------------------------------------------------------------
                path = paths[i]
                end_x, end_y = path[0][-1], path[1][-1]
                score = manhattan_distance(goal_state[0], goal_state[1], end_x, end_y)
                # --------------------------------------------------------------

                # Compute the "proximity to other colliding paths" score and
                # add it to the "distance from centerline" score.
                # The exact choice of objective function is up to you.
                for j in range(len(paths)):
                    if j == i:
                        continue
                    else:
                        if not collision_check_array[j]:
                            # TODO: INSERT YOUR CODE BETWEEN THE DASHED LINES
                            # --------------------------------------------------
                            # distance_to_colliding_path = manhattan_distance(paths[i][1][-1], paths[j][1][-1], paths[i][0][-1], paths[j][0][-1])
                            # distance_to_colliding_path = manhattan_distance(paths[i][1][-1], paths[i][0][-1],paths[j][1][-1],  paths[j][0][-1])
                            distance_to_colliding_path = paths[i][2][j]

                            score += self._weight * distance_to_colliding_path
                            # --------------------------------------------------

            # Handle the case of colliding paths.
            else:
                score = float('Inf')
                
            # Set the best index to be the path index with the lowest score
            if score < best_score:
                best_score = score
                best_index = i

        return best_index
