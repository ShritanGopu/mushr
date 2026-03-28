# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

def get_time_horizon(node):
    time_horizon = node.get_float("horizon/time", default=-1.0)
    if time_horizon > 0.0:
        return time_horizon
    else:
        dist_horizon = node.get_float("horizon/distance", default=2.0)
        vel = node.get_float("trajgen/desired_speed", default=1.0)
        return dist_horizon / vel


def get_distance_horizon(node):
    time_horizon = node.get_float("horizon/time", default=-1.0)
    if time_horizon > 0.0:
        vel = node.get_float("trajgen/desired_speed", default=1.0)
        return time_horizon * vel
    else:
        return node.get_float("horizon/distance", default=2.0)
