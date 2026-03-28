# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import os


def get_root_cache_dir(node):
    path = node.get_str("cache", default="~/.cache/mushr_rhc/")
    path = os.path.expanduser(path)

    if not os.path.isdir(path):
        os.mkdir(path)

    return path


def get_cache_dir(node, path):
    root = get_root_cache_dir(node)
    fullpath = os.path.join(root, path)

    if not os.path.isdir(fullpath):
        os.makedirs(fullpath)

    return fullpath


def get_cache_map_dir(node, map):
    return get_cache_dir(node, map.name)
