import numpy as np
import matplotlib.pyplot as plt


def get_generate_obstacles_func(obstacle_type):
    if obstacle_type == 'box':
        return generate_obstacles_box
    elif obstacle_type == 'circle':
        return generate_obstacles_circle
    else:
        raise NotImplementedError(f"Obstacle type {obstacle_type} not implemented")


def generate_obstacles_box(rnd_area, num_obstacles, obstacle_constraint, rng=None, seed=None):
    """

    @param rnd_area: x_min, x_max, y_min, y_max
    @param obstacle_constraint: min_width, min_height, max_width, max_height (for axis aligned bounding box obstacles)
    @return: List of obstacles (x0, x1, y0, y1)
    """
    rng = rng or np.random.default_rng(seed)
    x_min, x_max, y_min, y_max = rnd_area
    min_width, min_height, max_width, max_height = obstacle_constraint
    x_list = rng.uniform(x_min + max_width, x_max - max_width, num_obstacles)
    y_list = rng.uniform(y_min + max_height, y_max - max_height, num_obstacles)
    w_list = rng.uniform(min_width, max_width, num_obstacles)
    h_list = rng.uniform(min_height, max_height, num_obstacles)
    obstacles = [(x, x + w, y, y + h) for x, y, w, h in zip(x_list, y_list, w_list, h_list)]
    return obstacles


def generate_obstacles_circle(rnd_area, num_obstacles, obstacle_constraint, rng=None, seed=None):
    """

    @param rnd_area: x_min, x_max, y_min, y_max
    @param obstacle_constraint: min_radius, max_radius (for circle obstacles)
    @return: List of obstacles (x, y, r)
    """
    rng = rng or np.random.default_rng(seed)
    x_min, x_max, y_min, y_max = rnd_area
    min_radius, max_radius = obstacle_constraint
    x_list = rng.uniform(x_min + max_radius, x_max - max_radius, num_obstacles)
    y_list = rng.uniform(y_min + max_radius, y_max - max_radius, num_obstacles)
    r_list = rng.uniform(min_radius, max_radius, num_obstacles)
    obstacles = [(x, y, r) for x, y, r in zip(x_list, y_list, r_list)]
    return obstacles


def main():
    rnd_area = (0, 100, 0, 100)
    num_obstacle = 200
    min_width, min_height = 2, 2
    max_width, max_height = 5, 5
    obstacle_constraint = min_width, min_height, max_width, max_height
    # obstacles = generate_obstacles_box(rnd_area, num_obstacle, obstacle_constraint)
    obstacles = get_generate_obstacles_func("box")(rnd_area, num_obstacle, obstacle_constraint)
    # obstacle_constraint = 5, 2
    # obstacles = generate_obstacles_circle(rnd_area, num_obstacle, obstacle_constraint)
    plt.figure()
    for obstacle in obstacles:
        box = plt.Rectangle((obstacle[0], obstacle[2]), obstacle[1] - obstacle[0], obstacle[3] - obstacle[2],
                            fc='r')
        plt.gca().add_artist(box)
    plt.axis(rnd_area)
    plt.show()


if __name__ == '__main__':
    main()
