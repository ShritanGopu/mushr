# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import MapMetaData, OccupancyGrid
from nav_msgs.srv import GetMap

import librhc
import librhc.cost as cost
import librhc.model as model
import librhc.trajgen as trajgen
import librhc.types as types
import librhc.value as value
import librhc.worldrep as worldrep
import utils

motion_models = {"kinematic": model.Kinematics}

trajgens = {"tl": trajgen.TL, "dispersion": trajgen.Dispersion}

cost_functions = {"waypoints": cost.Waypoints}

value_functions = {"simpleknn": value.SimpleKNN}

world_reps = {"simple": worldrep.Simple}


class RHCBase(Node):
    def __init__(self, dtype):
        super().__init__("rhc_base")
        self.params = self
        self.dtype = dtype
        self._declare_base_parameters()
        self._load_base_parameters()

        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.cb_map_metadata,
            qos_profile=1
        )
        self.map_data = None

    def _declare_base_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("model_name", "kinematic"),
                ("trajgen_name", "tl"),
                ("cost_fn_name", "waypoints"),
                ("world_rep_name", "simple"),
                ("value_fn_name", "simpleknn"),
                ("map_file", "default"),
                ("static_map", "/static_map"),
            ],
        )

    def _load_base_parameters(self):
        self.model_name = self.get_parameter("model_name").value
        self.trajgen_name = self.get_parameter("trajgen_name").value
        self.cost_fn_name = self.get_parameter("cost_fn_name").value
        self.world_rep_name = self.get_parameter("world_rep_name").value
        self.value_fn_name = self.get_parameter("value_fn_name").value
        self.map_file = self.get_parameter("map_file").value
        self.static_map = self.get_parameter("static_map").value

    def get_str(self, path, default=None, global_=False):
        value = self._get_param_value(path, default, global_)
        return None if value is None else str(value)

    def get_dict(self, path, default=None, global_=False):
        return self._get_param_value(path, default, global_)

    def get_int(self, path, default=None, global_=False):
        value = self._get_param_value(path, default, global_)
        return None if value is None else int(value)

    def get_float(self, path, default=None, global_=False):
        value = self._get_param_value(path, default, global_)
        return None if value is None else float(value)

    def get_bool(self, path, default=None, global_=False):
        value = self._get_param_value(path, default, global_)
        return None if value is None else bool(value)

    def _get_param_value(self, path, default=None, global_=False):
        param_name = self._normalize_param_name(path, global_)
        if not self.has_parameter(param_name):
            self.declare_parameter(param_name, default)
        return self.get_parameter(param_name).value

    def _normalize_param_name(self, path, global_):
        if path.startswith("~/"):
            return path[2:]
        if path.startswith("~"):
            return path[1:]
        if path.startswith("/"):
            return path[1:]
        if global_:
            return path.lstrip("/")
        return path

    def load_controller(self):
        m = self.get_model()
        tg = self.get_trajgen(m)
        cf = self.get_cost_fn()

        return librhc.MPC(self, self.dtype, m, tg, cf)

    def get_model(self):
        if self.model_name not in motion_models:
            self.get_logger().fatal("model '{}' is not valid".format(self.model_name))

        return motion_models[self.model_name](self, self.dtype)

    def get_trajgen(self, model):
        if self.trajgen_name not in trajgens:
            self.get_logger().fatal("trajgen '{}' is not valid".format(self.trajgen_name))

        return trajgens[self.trajgen_name](self, self.dtype, model)

    def get_cost_fn(self):
        if self.cost_fn_name not in cost_functions:
            self.get_logger().fatal("cost_fn '{}' is not valid".format(self.cost_fn_name))

        if self.world_rep_name not in world_reps:
            self.get_logger().fatal("world_rep '{}' is not valid".format(self.world_rep_name))

        self.get_logger().info("Waiting for map metadata")
        while self.map_data is None:
            self.get_logger().info("Waiting for map metadata")
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Received map metadata")

        wr = world_reps[self.world_rep_name](
            self, self.dtype, self.map_data
        )

        if self.value_fn_name not in value_functions:
            self.get_logger().fatal("value_fn '{}' is not valid".format(self.value_fn_name))

        vf = value_functions[self.value_fn_name](
            self, self.dtype, self.map_data
        )
 
        return cost_functions[self.cost_fn_name](
           self, self.dtype, self.map_data, wr, vf
        )

    def cb_map_metadata(self, msg):
        default_map_name = "default"
        name = os.path.splitext(os.path.basename(self.map_file))[0]

        if name == default_map_name:
            self.get_logger().warn(
                "Default map name being used, will be corrupted on map change. "
                + "To fix, set '/map_file' parameter with map_file location"
            )
        self.get_logger().info("map data listening")
        metadata = msg.info
        x, y, angle = utils.rospose_to_posetup(metadata.origin)
        self.map_data = types.MapData(
            name=name,
            resolution=metadata.resolution,
            origin_x=x,
            origin_y=y,
            orientation_angle=angle,
            width=metadata.width,
            height=metadata.height,
            get_map_data=msg.data,
        )
        self.get_logger().info("mapdata converted to MapData")

    def get_map(self):
        srv_cli = self.create_client(GetMap, self.static_map)
        self.get_logger().debug("Waiting for map service")

        while not srv_cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interruped while waiting for the server.')
                return
            else:
                self.get_logger().info('Server not available, waiting again...')

        self.get_logger().debug("Map service started")

        map_msg = srv_cli.call(GetMap.Request())
        
        return map_msg.data
