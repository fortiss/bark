# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np

from modules.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderBehaviorModels
from modules.models.behavior.hypothesis.behavior_space.behavior_space import BehaviorSpace
from modules.runtime.scenario.scenario_generation.interaction_dataset_reader import behavior_from_track

from bark.models.behavior import *
from modules.runtime.commons.parameters import ParameterServer

  # this config reader defines behavior models with fixed type for all agents
class FixedBehaviorType(ConfigReaderBehaviorModels):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.param_servers = []

  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    model_type = config_param_object["ModelType", "Type of behavior model \
                used for all vehicles", "BehaviorIDMClassic"]
    model_params = config_param_object.AddChild("ModelParams")
    self.param_servers.append(model_params) # use the same param server for all models
    behavior_models = []
    behavior_model_types = []
    for _ in agent_states:
      bark_model, _ = self.model_from_model_type(model_type, model_params)
      behavior_models.append(bark_model)
      behavior_model_types.append(model_type)
    return behavior_models, {"behavior_model_types" : behavior_model_types}, config_param_object

  def model_from_model_type(self, model_type, params):
    bark_model = eval("{}(params)".format(model_type))
    return bark_model, params

  def get_param_servers(self):
    return self.param_servers


class BehaviorSpaceSampling(ConfigReaderBehaviorModels):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.param_servers = []

  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    behavior_space = BehaviorSpace(config_param_object)
    # now do the true sampling
    behavior_models = []
    behavior_model_types = []

    for _ in agent_states:
      model_params_sampled, model_type = behavior_space.sample_behavior_parameters(self.random_state)
      self.param_servers.append(model_params_sampled)
      bark_model, _ = self.model_from_model_type(model_type, model_params_sampled)
      behavior_models.append(bark_model)
      behavior_model_types.append(model_type)
    return behavior_models, {"behavior_model_types" : behavior_model_types}, config_param_object

  def model_from_model_type(self, model_type, params):
    bark_model = eval("{}(params)".format(model_type))
    return bark_model, params

  def get_param_servers(self):
    return self.param_servers


class InteractionDataBehaviors(ConfigReaderBehaviorModels):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.param_servers = []

  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    tracks = kwargs["tracks"]
    start_time = kwargs["start_time"]
    end_time = kwargs["end_time"]

    behavior_models = []
    behavior_model_types = []

    for idx, _ in enumerate(agent_states):
      track = tracks[idx]
      params = ParameterServer()
      behavior = behavior_from_track(track, params, start_time, end_time)
      self.param_servers.append(params)
      behavior_models.append(behavior)
      behavior_model_types.append("BehaviorStaticTrajectory")
    return behavior_models, {"behavior_model_types" : behavior_model_types}, config_param_object

  def get_param_servers(self):
    return self.param_servers