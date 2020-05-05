# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
import time
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.scenario.scenario_generation.config_with_ease import \
  LaneCorridorConfig, ConfigWithEase
from modules.runtime.runtime import Runtime
from modules.runtime.viewer.panda3d_viewer import Panda3dViewer


# parameters
param_server = ParameterServer()

# scenario
class CustomLaneCorridorConfig(LaneCorridorConfig):
  def __init__(self,
               road_ids=[16],
               lane_corridor_id=0,
               params=None):
    super(CustomLaneCorridorConfig, self).__init__(road_ids,
                                                   lane_corridor_id,
                                                   params)

  def position(self, world, min_s=10., max_s=150.):
    return super(CustomLaneCorridorConfig, self).position(world, min_s, max_s)

  def ds(self, s_min=20., s_max=35.):
    return np.random.uniform(s_min, s_max)

left_lane = CustomLaneCorridorConfig(lane_corridor_id=0, params=param_server)
right_lane = CustomLaneCorridorConfig(lane_corridor_id=1, params=param_server)
scenarios = \
  ConfigWithEase(num_scenarios=5,
                 map_file_name="modules/runtime/tests/data/city_highway_straight.xodr",
                 random_seed=0,
                 params=param_server,
                 lane_corridor_configs=[left_lane, right_lane])

# viewer
viewer = MPViewer(params=param_server,
                  use_world_bounds=True)
viewer = Panda3dViewer(params=param_server,
                       use_world_bounds=True,
                       x_range=[-40, 40],
                       y_range=[-40, 40],
                       follow_agent_id=1)


# gym like interface
env = Runtime(step_time=0.2,
              viewer=viewer,
              scenario_generator=scenarios,
              render=True)
      
sim_step_time = param_server["simulation"]["step_time",
                                          "Step-time used in simulation",
                                          0.05]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster",
                                                  1]
for _ in range(0, 3):
  env.reset()
  for _ in range(0, 10):
    env.step()
    time.sleep(sim_step_time/sim_real_time_factor)
