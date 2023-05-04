import numpy as np
from rrt_circle import RRT
from numpy import searchsorted, argsort, zeros, bool_, nonzero
from binary_search import binary_search_left, binary_search_right


# from numba import jit

# @jit("i8[:](i8,i8[:],i8[:],i8[:],i8[:])", cache=False, nopython=True)
def intersection_np_nonzero(num_obstacles, list1, list2, list3, list4):
    c1 = zeros(num_obstacles, dtype=bool_)
    c2 = zeros(num_obstacles, dtype=bool_)
    c3 = zeros(num_obstacles, dtype=bool_)
    c4 = zeros(num_obstacles, dtype=bool_)

    c1[list1] = True
    c2[list2] = True
    c3[list3] = True
    c4[list4] = True
    return nonzero(c1 & c2 & c3 & c4)[0]


class RRT_COF(RRT):
    def __init__(self, start, goal, obstacles, play_area, rnd_area=None, max_iter=1000, step_len=5,
                 robot_radius=0.0, goal_sample_rate=0.05, rng=None, seed=None
                 ):
        super().__init__(start, goal, obstacles, play_area, rnd_area,
                         max_iter, step_len, robot_radius, goal_sample_rate, rng, seed)
        self.init_prior_info()

    def init_prior_info(self):
        obstacles = np.array(self.obstacles)
        obstacles_left_bound = obstacles[:, 0] - obstacles[:, 2]
        obstacles_right_bound = obstacles[:, 0] + obstacles[:, 2]
        obstacles_bottom_bound = obstacles[:, 1] - obstacles[:, 2]
        obstacles_top_bound = obstacles[:, 1] + obstacles[:, 2]
        self.obstacles_left_bound_sorted_idx = argsort(obstacles_left_bound)
        self.obstacles_right_bound_sorted_idx = argsort(obstacles_right_bound)
        self.obstacles_bottom_bound_sorted_idx = argsort(obstacles_bottom_bound)
        self.obstacles_top_bound_sorted_idx = argsort(obstacles_top_bound)
        self.obstacles_left_bound_sorted = obstacles_left_bound[self.obstacles_left_bound_sorted_idx]
        self.obstacles_right_bound_sorted = obstacles_right_bound[self.obstacles_right_bound_sorted_idx]
        self.obstacles_bottom_bound_sorted = obstacles_bottom_bound[self.obstacles_bottom_bound_sorted_idx]
        self.obstacles_top_bound_sorted = obstacles_top_bound[self.obstacles_top_bound_sorted_idx]
        self.obstacles = obstacles
        self.num_obstacles = obstacles.shape[0]
        self.num_obstacle_candidate_thr = 10

    def _get_obstacles_idx_about_bound(self, left, right, bottom, top):
        num_obstacles = self.num_obstacles
        robot_radius = self.robot_radius
        num_obstacle_candidate_thr = self.num_obstacle_candidate_thr
        # left_start = searchsorted(self.obstacles_right_bound_sorted, left - robot_radius, side='left')
        left_start = binary_search_left(self.obstacles_right_bound_sorted, left - robot_radius)
        candidate_l = self.obstacles_right_bound_sorted_idx[left_start:]
        if left_start >= num_obstacles - num_obstacle_candidate_thr:
            return candidate_l

        # right_end = searchsorted(self.obstacles_left_bound_sorted, right + robot_radius, side='right')
        right_end = binary_search_right(self.obstacles_left_bound_sorted, right + robot_radius)
        candidate_r = self.obstacles_left_bound_sorted_idx[:right_end]
        if right_end <= num_obstacle_candidate_thr:
            return candidate_r

        # bottom_start = searchsorted(self.obstacles_top_bound_sorted, bottom - robot_radius, side='left')
        bottom_start = binary_search_left(self.obstacles_top_bound_sorted, bottom - robot_radius)
        candidate_b = self.obstacles_top_bound_sorted_idx[bottom_start:]
        if bottom_start >= num_obstacles - num_obstacle_candidate_thr:
            return candidate_b

        # top_end = searchsorted(self.obstacles_bottom_bound_sorted, top + robot_radius, side='right')
        top_end = binary_search_right(self.obstacles_bottom_bound_sorted, top + robot_radius)
        candidate_t = self.obstacles_bottom_bound_sorted_idx[:top_end]
        if top_end <= num_obstacle_candidate_thr:
            return candidate_t

        # candidate_list = list(set(candidate_l) & set(candidate_r) & set(candidate_b) & set(candidate_t))
        candidate_list = intersection_np_nonzero(num_obstacles, candidate_l, candidate_r, candidate_b, candidate_t)
        return candidate_list

    def get_obstacles_about_node(self, node):
        x = node.x
        y = node.y
        candidate_ids = self._get_obstacles_idx_about_bound(x, x, y, y)
        return self.obstacles[candidate_ids]

    def get_obstacles_about_segment(self, from_node, to_node):
        from_x = from_node.x
        from_y = from_node.y
        to_x = to_node.x
        to_y = to_node.y
        left, right = (from_x, to_x) if from_x < to_x else (to_x, from_x)
        bottom, top = (from_y, to_y) if from_y < to_y else (to_y, from_y)
        candidate_ids = self._get_obstacles_idx_about_bound(left, right, bottom, top)
        return self.obstacles[candidate_ids]
