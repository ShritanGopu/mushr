# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import matplotlib.cm as cm
import matplotlib.colors as colors
import rclpy
import torch
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

_traj_pub = None

def viz_paths_cmap(node, poses, costs, ns="paths", cmap="plasma", scale=0.03):
    global _traj_pub
    if _traj_pub is None:
        _traj_pub = node.create_publisher(MarkerArray, "~/debug/viz_rollouts", 100)

    max_c = torch.max(costs)
    min_c = torch.min(costs)

    norm = colors.Normalize(vmin=min_c, vmax=max_c)

    cmap = cm.get_cmap(name=cmap)

    def colorfn(cost):
        r, g, b, a = 0.0, 0.0, 0.0, 1.0
        if cost == min_c:
            return r, g, b, a
        col = cmap(norm(cost))
        r, g, b = col[0], col[1], col[2]
        if len(col) > 3:
            a = col[3]
        return r, g, b, a

    return viz_paths(node, poses, costs, colorfn, ns, scale)


def viz_paths(node, poses, costs, colorfn, ns="paths", scale=0.03):
    """
    poses should be an array of trajectories to plot in rviz
    costs should have the same dimensionality as poses.size()[0]
    colorfn maps a point to an rgb tuple of colors
    """
    assert poses.size()[0] == costs.size()[0]

    markers = MarkerArray()

    for i, (traj, cost) in enumerate(zip(poses, costs)):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rclpy.time.Time().to_msg()
        m.ns = ns
        m.id = i
        m.type = m.LINE_STRIP
        m.action = m.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.color.r, m.color.g, m.color.b, m.color.a = colorfn(cost)

        for t in traj:
            p = Point()
            p.x = float(t[0])
            p.y = float(t[1])
            m.points.append(p)

        markers.markers.append(m)

    for i in range(len(poses), node.get_int("K"), 1):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rclpy.time.Time().to_msg()
        m.ns = ns
        m.id = i
        m.type = m.LINE_STRIP
        m.action = m.DELETE
        markers.markers.append(m)

    _traj_pub.publish(markers)
