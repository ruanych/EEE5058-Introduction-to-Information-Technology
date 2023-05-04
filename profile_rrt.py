import os
import time
import numpy as np
from itertools import product
from cProfile import Profile
import pstats
from env_util import get_generate_obstacles_func

import plotly
import plotly.express as px
import plotly.graph_objects as go


def func_timing(pyfunc, func_kwargs, observed_func_name, n_runs=10):
    """
    Get n_calls, tot_time, cum_time of the observed functions

    @param pyfunc: the function to be profiled
    @param func_kwargs: the kwargs of the function to be profiled
    @param observed_func_name: the function name to be observed
    @param n_runs: the number of runs to get the average time cost
    @return: a dict of the average time cost of the observed functions,
        e.g. {"func_name": {"n_calls": 0, "tot_time": 0, "cum_time": 0}}
    """
    # func_name, n_calls, tot_time, cum_time
    observed_func_data_dict = {_func_name: {"n_calls": [], "tot_time": [], "cum_time": []}
                               for _func_name in observed_func_name}

    for i in range(n_runs):
        prof = Profile()
        prof.enable()
        pyfunc(**func_kwargs)
        prof.disable()
        ps = pstats.Stats(prof)
        for _func, (_cc, _n_calls, _tot_time, _cum_time, _callers) in ps.stats.items():
            # cc [int] primitive calls, the number of times the function was called, excluding recursive calls
            # n_calls [int] function calls, the number of times the function was called, including recursive calls
            # tot_time [float] execution time, excluding any time spent in sub-functions
            # cum_time [float] cumulative execution time, including time spent in sub-functions
            # callers [dict], key is (filename,lineno,function_name)
            _file_name, _line_number, _func_name = _func
            if _func_name in observed_func_name:
                _observed_func_data = observed_func_data_dict[_func_name]
                _observed_func_data["n_calls"].append(_n_calls)
                _observed_func_data["tot_time"].append(_tot_time)
                _observed_func_data["cum_time"].append(_cum_time)
    result = {}
    for _func_name, _observed_func_data in observed_func_data_dict.items():
        result[_func_name] = {}
        for _key, _value in _observed_func_data.items():
            _avg_value = np.mean(_value).item() if len(_value) > 0 else 0
            result[_func_name][_key] = _avg_value
    return result


def plot_cum_time_stat_inter(algo_name_list, time_data_dict, num_obstacle_list, observed_func_name_list,
                             time_cost_reduce_func=np.mean):
    mask_idx = len(num_obstacle_list)
    # x axis: num_obstacle, y axis: cum_time
    fig = go.Figure(layout={
        "title": "rrt vs rrt_cof",
        "xaxis_title": "Number of Obstacles",
        "yaxis_title": "Cumulative Time (s)",
        "template": "plotly_white",
    })
    for _algo_name, _func_name in product(algo_name_list, observed_func_name_list):
        fig.add_trace(go.Scatter(
            x=num_obstacle_list,
            y=time_cost_reduce_func(time_data_dict[_algo_name, _func_name][:mask_idx, :], axis=1),
            mode='lines+markers',
            name=_algo_name + " - " + _func_name))
    return fig


def plot_cum_time_stat(algo_name_list, time_data_dict, num_obstacle_list, observed_func_name_list,
                       file_name="save_dir/profile_rrt_cum_time.html",
                       time_cost_reduce_func=np.mean):
    fig = plot_cum_time_stat_inter(algo_name_list, time_data_dict, num_obstacle_list, observed_func_name_list,
                                   time_cost_reduce_func)
    fig.write_html(file_name)


def rrt_timing(algo_name_list, algo_class_list,
               num_obstacle_interval, num_obstacle_step,
               map_cnt_per_obstacle_num, num_runs_per_map,
               seed=None,
               plot_file=None, dump_data_file=None):
    # map config
    play_area = (0, 600, 0, 600)
    rnd_area = (10, 590, 10, 490)

    # for axis aligned bounding box obstacles, min_width, min_height, max_width, max_height
    box_obstacle_constraint = (2, 6, 2, 6)
    # for circle obstacles, min_radius, max_radius
    circle_obstacle_constraint = (2, 6)
    obstacle_constraint_dict = {"box": box_obstacle_constraint, "circle": circle_obstacle_constraint}

    # robot config
    start = (1, 1)
    goal = (599, 599)
    step_len = 3
    robot_radius = 1

    max_iter = 50000
    goal_sample_rate = 0.05

    # timing config
    num_obstacle_list = list(range(num_obstacle_interval[0], num_obstacle_interval[1] + 1, num_obstacle_step))
    observed_func_name_list = ['is_collision_free',
                               'get_obstacles_about_segment', 'get_obstacles_about_node',
                               'init_prior_info']

    metric_name_list = ["cum_time", "tot_time", "n_calls"]

    # func time statistic
    algo_cnt = len(algo_class_list)
    time_data_dict = {}  # dict_key: metric_name -> dict_key: (algo_name, func_name) -> ndarray: (num_obstacle, map_cnt)
    for _metric_name in metric_name_list:
        time_data_dict[_metric_name] = {
            (_algo_name, _func_name): np.zeros((len(num_obstacle_list), map_cnt_per_obstacle_num))
            for _algo_name, _func_name in product(algo_name_list, observed_func_name_list)
        }

    # algo statistic
    algo_statistic_attr_list = ["num_obstacles_checked_avg", "iter_used"]
    algo_statistic_data_dict = {}  # dict_key: attr_name -> dict_key: algo_name -> ndarray: (num_obstacle, map_cnt)
    for _attr_name in algo_statistic_attr_list:
        algo_statistic_data_dict[_attr_name] = {
            _algo_name: np.zeros((len(num_obstacle_list), map_cnt_per_obstacle_num))
            for _algo_name in algo_name_list
        }

    time_cost_reduce_func = np.mean
    # time_cost_reduce_func = np.median

    rng_seed = seed if seed is not None else int(time.time())

    for obs_num_id, num_obstacle in enumerate(num_obstacle_list):
        for map_id in range(map_cnt_per_obstacle_num):
            print("num_obstacle: ", num_obstacle, "  , map_id: ", map_id)
            rng_seed += 1
            for _algo_name, algo_class in zip(algo_name_list, algo_class_list):
                # build obstacles
                obstacle_type = algo_class.obstacle_type
                obstacle_gen_func = get_generate_obstacles_func(obstacle_type)
                obstacle_constraint = obstacle_constraint_dict[obstacle_type]
                obstacles = obstacle_gen_func(rnd_area, num_obstacle, obstacle_constraint, seed=rng_seed)

                # build algo instance and run timing
                algo_inst = algo_class(start, goal, obstacles, play_area, rnd_area,
                                       max_iter=max_iter, step_len=step_len, robot_radius=robot_radius,
                                       goal_sample_rate=goal_sample_rate, seed=rng_seed)
                algo_time_data = func_timing(algo_inst.planning, {"seed": rng_seed}, observed_func_name_list,
                                             n_runs=num_runs_per_map)

                # record time data
                for _m_name in metric_name_list:
                    _m_dict = time_data_dict[_m_name]
                    for _func_name in observed_func_name_list:
                        _m_dict[_algo_name, _func_name][obs_num_id, map_id] = algo_time_data[_func_name][_m_name]

                # record algo statistic
                for _attr in algo_statistic_attr_list:
                    algo_statistic_data_dict[_attr][_algo_name][obs_num_id, map_id] = getattr(algo_inst, _attr)

                if not getattr(algo_inst, "found"):
                    print("No path found, num_obstacle=", num_obstacle, "  , map_id: ", map_id,
                          " , algo_name: ", _algo_name)

        if plot_file:
            plot_cum_time_stat(algo_name_list, time_data_dict['cum_time'], num_obstacle_list[:obs_num_id + 1],
                               observed_func_name_list, file_name=plot_file,
                               time_cost_reduce_func=time_cost_reduce_func)

        # save data
        if dump_data_file is not None:
            time_data_dict_desc = "dict_key: metric_name -> dict_key: (algo_name, func_name) -> ndarray: (num_obstacle, map_cnt)"
            algo_statistic_data_dict_desc = "dict_key: attr_name -> dict_key: algo_name -> ndarray: (num_obstacle, map_cnt)"

            data_dumped = {"time_data_dict_desc": time_data_dict_desc,
                           "algo_statistic_data_dict_desc": algo_statistic_data_dict_desc,
                           "num_obstacle_list": num_obstacle_list,
                           "algo_name_list": algo_name_list,
                           "observed_func_name_list": observed_func_name_list,
                           "metric_name_list": metric_name_list,
                           "algo_statistic_attr_list": algo_statistic_attr_list,
                           "time_data_dict": time_data_dict,
                           "algo_statistic_data_dict": algo_statistic_data_dict
                           }
            np.save(dump_data_file, data_dumped)


def main(save_dir, seed=None):
    from rrt_cof import RRT_COF
    from rrt import RRT
    from rrt_circle import RRT as RRT_CIRCLE
    from rrt_cof_circle import RRT_COF as RRT_COF_CIRCLE

    algo_name_list = ["RRT", "RRT_COF"]
    algo_class_list = [RRT, RRT_COF]

    num_obstacle_step = 10
    num_obstacle_interval = (num_obstacle_step, 1000)
    map_cnt_per_obstacle_num = 10
    num_runs_per_map = 5
    rrt_timing(algo_name_list, algo_class_list,
               num_obstacle_interval, num_obstacle_step,
               map_cnt_per_obstacle_num, num_runs_per_map,
               seed=seed,
               plot_file=save_dir + "/profile_rrt_cum_time.html",
               dump_data_file=save_dir + "/profile_rrt_dump_data.npy")

    algo_class_list = [RRT_CIRCLE, RRT_COF_CIRCLE]
    num_obstacle_step = 2
    num_obstacle_interval = (num_obstacle_step, 200)
    map_cnt_per_obstacle_num = 10
    num_runs_per_map = 5
    rrt_timing(algo_name_list, algo_class_list,
               num_obstacle_interval, num_obstacle_step,
               map_cnt_per_obstacle_num, num_runs_per_map,
               seed=seed,
               plot_file=save_dir + "/profile_rrt_circle_cum_time.html",
               dump_data_file=save_dir + "/profile_rrt_circle_dump_data.npy")


if __name__ == '__main__':
    time_str = time.strftime("%Y%m%d-%H%M%S")
    save_dir = "save_dir/" + time_str
    os.makedirs(save_dir, exist_ok=True)
    main(save_dir)
