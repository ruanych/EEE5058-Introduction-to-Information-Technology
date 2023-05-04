import time
from rrt_cof_circle import RRT_COF
from rrt_circle import RRT
import numpy as np
import env_util

from cProfile import Profile
import pstats

import argparse


def main(map_id=1, visualize=True):
    seed = 2023
    # seed = int(time.time())

    # test1
    play_area = (0, 200, 0, 200)
    rnd_area = (65, 195, 65, 195)
    num_obstacle = 14
    circle_obstacle_constraint = (2, 5)
    obstacles = env_util.generate_obstacles_circle(rnd_area, num_obstacle, circle_obstacle_constraint, seed=seed)
    if map_id == 1:
        obstacles.append((55, 55, 5))  # Map01
    else:
        obstacles.append((55, 55, 50))  # Map02

    start = (1, 1)
    goal = (199, 199)
    step_len = 1
    robot_radius = 1

    max_iter = 20000
    goal_sample_rate = 0.05

    prof_rrt = Profile()
    prof_rrt.enable()
    rrt = RRT(start, goal, obstacles, play_area, rnd_area, max_iter=max_iter,
              step_len=step_len, robot_radius=robot_radius, goal_sample_rate=goal_sample_rate,
              seed=seed
              )

    start_time = time.time()
    rrt.planning(seed=seed)
    rrt_time = time.time() - start_time

    prof_rrt.disable()
    stats_rrt = pstats.Stats(prof_rrt).strip_dirs()
    stats_profile_rrt = stats_rrt.get_stats_profile()
    is_collision_free_time_rrt = stats_profile_rrt.func_profiles["is_collision_free"].cumtime

    prof_rrt_cof = Profile()
    prof_rrt_cof.enable()

    rrt_cof = RRT_COF(start, goal, obstacles, play_area, rnd_area, max_iter=max_iter,
                      step_len=step_len, robot_radius=robot_radius, goal_sample_rate=goal_sample_rate,
                      seed=seed
                      )

    start_time = time.time()
    rrt_cof.planning()
    rrt_cof_time = time.time() - start_time

    prof_rrt_cof.disable()
    stats_rrt_cof = pstats.Stats(prof_rrt_cof).strip_dirs()
    stats_profile_rrt_cof = stats_rrt_cof.get_stats_profile()
    is_collision_free_time_rrt_cof = stats_profile_rrt_cof.func_profiles["is_collision_free"].cumtime
    init_prior_info_time_rrt_cof = stats_profile_rrt_cof.func_profiles["init_prior_info"].cumtime

    print("num_obstacle: ", len(obstacles))
    print("num_obstacles_checked_avg: RRT ", rrt.num_obstacles_checked_avg,
          " , RRT_COF ", rrt_cof.num_obstacles_checked_avg)

    same = np.array_equal(rrt.path, rrt_cof.path)
    print("Path Found : ", rrt.found, ", Path Same: ", same)
    print("RRT path len: ", len(rrt.path), ", iter_used: ", rrt.iter_used)

    print("Total Time: RRT ", rrt_time, ", RRT_COF ", rrt_cof_time)
    print(f"is_collision_free_time: RRT {is_collision_free_time_rrt:.5f}, RRT_COF {is_collision_free_time_rrt_cof:.5f}"
          f", RRT_COF/RRT: {is_collision_free_time_rrt_cof / is_collision_free_time_rrt:.5f}"
          f", speedup: {(is_collision_free_time_rrt - is_collision_free_time_rrt_cof) / is_collision_free_time_rrt:.5f}")

    # print("\n init_prior_info_time_rrt_cof: ", init_prior_info_time_rrt_cof)

    if visualize:
        if len(rrt_cof.path) > 0:
            rrt_cof.visualize()
        elif len(rrt.path) > 0:
            rrt.visualize()


if __name__ == '__main__':
    # choose map01 or map02 by commend line argument
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=int, default=1)
    args = parser.parse_args()
    map_id = args.map

    main(map_id=map_id, visualize=True)
