# -*- coding: utf-8 -*-

import numpy as np
import math

import config



def generate_points_on_line(begin, end, steps):
    length = end - begin
    div = float(length) / steps

    points = [begin + div * i for i in range(steps + 1)]
    return points


def __check_range(range, min, max, step):
    range[0] = min if range[0] < min else range[0]
    range[1] = max if range[1] > max else range[1]
    range[2] = step if range[2] < step else range[2]


def __check_approach_config(config):
    __check_range(config['xyz']['theta'], 0, 179, 1)
    __check_range(config['xyz']['phi'], -180, 180, 1)
    __check_range([0] + config['xyz']['radius'], 0, 100, 1)
    if config['abc'] is not None:
        __check_range([0] + config['abc']['end'], 0, 0, 0)


def __generate_route(phi, theta, radius_points):
    route = []
    for r in radius_points:
        x = r * math.sin(math.radians(theta)) * math.cos(math.radians(phi))
        y = r * math.sin(math.radians(theta)) * math.sin(math.radians(phi))
        z = r * math.cos(math.radians(theta))
        route.append([x, y, z])
    return route


def generate_approach_routes_xyz(
        angle_theta_range=(0, 90, 9),
        angle_phi_range=(-180, 180, 30),
        radius=(0, 0.5, 50),
        digits=10):
    theta_points = generate_points_on_line(*angle_theta_range)
    phi_points = generate_points_on_line(*angle_phi_range)
    if phi_points[0] == -180 and phi_points[-1] == 180:
        phi_points = phi_points[:-1]

    radius_points = generate_points_on_line(0, *radius)

    if theta_points[0] == 0:
        routes = [__generate_route(0, 0, radius_points)]
        theta_points = theta_points[1:]
    else:
        routes = []

    for t in theta_points:
        for p in phi_points:
            routes.append(__generate_route(p, t, radius_points))

    return np.round(np.array(routes), digits)


def __check_insert_config(config):
    config['end'] = [p if p >= 0 else 0 for p in config['end']]
    config['steps'] = config['steps'] if config['steps'] >= 0 else 0


def generate_waypoints(begin, end, steps):
    begin_arr = np.array(begin)
    end_arr = np.array(end)
    div = (end_arr - begin_arr) / float(steps)
    return np.array([div * i for i in range(steps + 1)])


def generate_routes(cfg_approach, cfg_insert):
    # 設定確認
    __check_approach_config(cfg_approach)
    __check_insert_config(cfg_insert)
    # 経路（xyz）の生成
    approach_routes = generate_approach_routes_xyz(
        cfg_approach['xyz']['theta'],
        cfg_approach['xyz']['phi'],
        cfg_approach['xyz']['radius']
    )
    insert_route = generate_waypoints(
        [0, 0, 0],
        cfg_insert['end'][:3],
        cfg_insert['steps']
    )
    insert_routes = np.tile(insert_route, (approach_routes.shape[0], 1, 1))
    insert_routes = insert_routes[:, :-1, :]
    approach_routes = approach_routes[:, :, :] + insert_route[-1]
    if cfg_approach['abc'] is not None:
        # 経路（abc）の生成
        insert_abc = np.zeros((*insert_routes.shape[:2],3))
        insert_routes = np.append(insert_routes, insert_abc, axis=2)
        approach_abc = generate_waypoints(
            [0,0,0],
            cfg_approach['abc']['end'],
            cfg_approach['xyz']['radius'][1]
        )
        approach_abc = np.tile(approach_abc, (approach_routes.shape[0],1,1))
        approach_routes = np.append(approach_routes, approach_abc, axis=2)

    return np.hstack( (insert_routes, approach_routes) )




if __name__ == '__main__':
    import pickle
    routes = generate_routes(
        config.CONFIG_GEN_ROUTES['approach'],
        config.CONFIG_GEN_ROUTES['insert']
    )

    # for i, route in enumerate(routes):
    #     np.savetxt('route_{}.csv'.format(i), route, delimiter=',')
    with open('routes.pkl', 'wb') as f:
        pickle.dump(routes, f)

