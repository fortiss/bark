// Copyright (c) 2024 fortiss GmbH
//
// Authors: Esra Acar-Celik and Xiangzhong Liu
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_STL_LABELS_STL_DISCRETE_TIME_SPECIFICATION_HPP_
#define BARK_WORLD_EVALUATION_STL_LABELS_STL_DISCRETE_TIME_SPECIFICATION_HPP_

#include <iostream>

#include "bark/world/objects/object.hpp"
#include "bark/world/world.hpp"
#include "bark/commons/transformation/frenet_state.hpp"
#include "python_interpreter.hpp"

namespace bark {
namespace world {
namespace evaluation {

using bark::commons::transformation::FrenetPosition;
using bark::world::objects::AgentPtr;
using bark::world::FrontRearAgents;

class StlDiscreteTimeSpecificationWrapper {
public:
  StlDiscreteTimeSpecificationWrapper();

  ~StlDiscreteTimeSpecificationWrapper() {
    // Py_XDECREF(pInstance);
  }

  bool InitializePyObj();

  void DeclareVar(std::string var_name, std::string var_type);

  void SetSamplingPeriod(int signal_sampling_period, std::string unit, double tolerance);

  void SetStringAttribute(std::string attr_name, std::string attr_value);
  
  void CallMethodWithNoArg(std::string method_name);

  double UpdateStlSpec(double stl_spec_timestep, std::vector<std::pair<std::string, double>> dataset) const;

private:
  static PyObject* pModule;

  PyObject* pInstance;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_STL_LABELS_STL_DISCRETE_TIME_SPECIFICATION_HPP_
