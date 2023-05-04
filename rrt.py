import numpy as np
import math
import matplotlib.pyplot as plt


class RRT:
    obstacle_type = 'box'

    class Node:
        def __init__(self, x, y, parent=None):
            self.x = x
            self.y = y
            self.parent = parent

        def __str__(self):
            return f'({self.x}, {self.y})'

        def __repr__(self):
            return '<%(cls)s - %(data)s>' % \
                dict(cls=self.__class__.__name__, data=f'{self.__str__()}, parent={self.parent}')

    def __init__(self, start, goal, obstacles, play_area, rnd_area=None,
                 max_iter=1000, step_len=5, robot_radius=0.0, goal_sample_rate=0.05,
                 rng=None, seed=None
                 ):
        self.start = self.Node(start[0], start[1])
        self.goal = self.Node(goal[0], goal[1])
        self.obstacles = obstacles  # [[x_min, x_max, y_min, y_max], ...]
        self.play_area = play_area  # x_min, x_max, y_min, y_max
        if rnd_area:
            self.rnd_area = [min(rnd_area[0], play_area[0]),
                             max(rnd_area[1], play_area[1]),
                             min(rnd_area[2], play_area[2]),
                             max(rnd_area[3], play_area[3])]
        else:
            self.rnd_area = play_area
        self.max_iter = max_iter
        self.step_len = step_len
        self.robot_radius = robot_radius
        self.goal_sample_rate = goal_sample_rate
        self.rng = rng or np.random.default_rng(seed=seed)

        self.nodes = []
        self.path = []
        self.found = False
        self.iter_used = 0
        self.num_obstacles_checked_avg = 0

    def get_obstacles_about_node(self, node):
        return self.obstacles

    def get_obstacles_about_segment(self, from_node, to_node):
        return self.obstacles

    def is_in_obstacle(self, node, obstacle):
        # Axis Aligned Bounding Box
        x = node.x
        y = node.y
        robot_radius = self.robot_radius
        return obstacle[0] - robot_radius <= x <= obstacle[1] + robot_radius and \
            obstacle[2] - robot_radius <= y <= obstacle[3] + robot_radius

    def is_in_obstacles(self, node):
        obstacle_list = self.get_obstacles_about_node(node)
        node_min_x = node.x - self.robot_radius
        node_max_x = node.x + self.robot_radius
        node_min_y = node.y - self.robot_radius
        node_max_y = node.y + self.robot_radius
        for obstacle in obstacle_list:
            if obstacle[0] <= node_max_x and node_min_x <= obstacle[1] and \
                    obstacle[2] <= node_max_y and node_min_y <= obstacle[3]:
                return True
        return False

    def is_goal_reached(self, node):
        dx = node.x - self.goal.x
        dy = node.y - self.goal.y
        return dx * dx + dy * dy <= self.step_len * self.step_len

    def sample_rnd_node(self):
        if self.rng.random() > self.goal_sample_rate:
            rnd = self.Node(
                self.rng.uniform(self.rnd_area[0], self.rnd_area[1]),
                self.rng.uniform(self.rnd_area[2], self.rnd_area[3]))
        else:
            rnd = self.Node(self.goal.x, self.goal.y)
        return rnd

    def sample_free(self):
        while True:
            if self.rng.random() > self.goal_sample_rate:
                rnd = self.Node(
                    self.rng.uniform(self.rnd_area[0], self.rnd_area[1]),
                    self.rng.uniform(self.rnd_area[2], self.rnd_area[3]))
                if not self.is_in_obstacles(rnd):
                    break
            else:
                rnd = self.Node(self.goal.x, self.goal.y)
                break
        return rnd

    def nearest_node(self, rnd_node):
        x = rnd_node.x
        y = rnd_node.y
        dists = [(node.x - x) ** 2 + (node.y - y) ** 2 for node in self.nodes]
        min_dist = min(dists)
        nearest_node = self.nodes[dists.index(min_dist)]
        return nearest_node

    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    def steer(self, from_node, to_node):
        d, theta = self.calc_distance_and_angle(from_node, to_node)
        expand_d = min(d, self.step_len)
        x = from_node.x + expand_d * math.cos(theta)
        y = from_node.y + expand_d * math.sin(theta)
        new_node = self.Node(x, y, from_node)
        return new_node

    def is_collision_free(self, from_node, to_node):
        # obstacle_list = self.get_obstacles_about_node(to_node)
        #
        # for obstacle in obstacle_list:
        #     if self.is_in_obstacle(to_node, obstacle):
        #         return False

        obstacle_list = self.get_obstacles_about_segment(from_node, to_node)
        self.num_obstacles_checked_avg += len(obstacle_list)
        if len(obstacle_list) == 0:
            return True

        from_x = from_node.x
        from_y = from_node.y
        to_x = to_node.x
        to_y = to_node.y
        node_min_x, node_max_x = (from_x, to_x) if from_x < to_x else (to_x, from_x)
        node_min_y, node_max_y = (from_y, to_y) if from_y < to_y else (to_y, from_y)
        robot_radius = self.robot_radius
        node_min_x -= robot_radius
        node_max_x += robot_radius
        node_min_y -= robot_radius
        node_max_y += robot_radius
        for obstacle in obstacle_list:
            if obstacle[0] <= node_max_x and obstacle[1] >= node_min_x and \
                    obstacle[2] <= node_max_y and obstacle[3] >= node_min_y:
                return False
        return True

    def generate_path(self, last_node):
        path = [[self.goal.x, self.goal.y]]
        while last_node:
            path.append([last_node.x, last_node.y])
            last_node = last_node.parent
        path.reverse()
        return path

    def planning(self, rng=None, seed=None):
        if rng is not None or seed is not None:
            self.rng = rng or np.random.default_rng(seed)

        self.nodes = [self.start]
        self.found = False
        self.iter_used = self.max_iter
        self.path = []
        for i in range(self.max_iter):
            # rnd_node = self.sample_free()
            rnd_node = self.sample_rnd_node()
            nearest_node = self.nearest_node(rnd_node)
            new_node = self.steer(nearest_node, rnd_node)
            if self.is_collision_free(nearest_node, new_node):
                self.nodes.append(new_node)
                if self.is_goal_reached(new_node) and self.is_collision_free(new_node, self.goal):
                    self.found = True
                    self.path = self.generate_path(new_node)
                    self.iter_used = i
                    break
        self.num_obstacles_checked_avg /= self.iter_used

    def visualize(self):
        plt.figure()
        for obstacle in self.obstacles:
            box = plt.Rectangle((obstacle[0], obstacle[2]), obstacle[1] - obstacle[0], obstacle[3] - obstacle[2],
                                fc='r', fill=False)
            plt.gca().add_artist(box)
        for node in self.nodes:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'k-', linewidth=0.5)
        if self.found:
            for i in range(len(self.path) - 1):
                plt.plot([self.path[i][0], self.path[i + 1][0]], [self.path[i][1], self.path[i + 1][1]], 'g-',
                         linewidth=2)
        plt.plot(self.start.x, self.start.y, 'bo', markersize=5)
        plt.plot(self.goal.x, self.goal.y, 'bo', markersize=5)
        plt.axis('equal')
        plt.axis(self.play_area)
        plt.show()
