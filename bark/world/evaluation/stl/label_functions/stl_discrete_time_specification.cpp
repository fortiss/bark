// Copyright (c) 2024 fortiss GmbH
//
// Authors: Esra Acar-Celik and Xiangzhong Liu
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "stl_discrete_time_specification.hpp"
#include <Python.h>

namespace bark {
namespace world {
namespace evaluation {

py::module* StlDiscreteTimeSpecificationWrapper::pModule_ = nullptr;

StlDiscreteTimeSpecificationWrapper::StlDiscreteTimeSpecificationWrapper() {
  if (!Py_IsInitialized()) {
    py::scoped_interpreter guard{};  
  }

  if (pModule_ == nullptr) {      
    pModule_ = new py::module(py::reinterpret_borrow<py::module>(py::module::import("rtamt")));
    if (pModule_ == nullptr) {
      std::cerr << "Failed to load rtamt Python module" << std::endl;
    } 
  }
}

bool StlDiscreteTimeSpecificationWrapper::InitializePyObj() {
  if (pModule_) {   
    pInstance_ = new py::object(pModule_->attr("StlDiscreteTimeSpecification")());
    if (pInstance_) {        
      // std::cout << "Successfully initialized StlDiscreteTimeSpecification object!" << std::endl;
      return true;
    }       
  }
    
  return false;
}

void StlDiscreteTimeSpecificationWrapper::DeclareVar(std::string var_name, std::string var_type) {
  if (pInstance_) {
    pInstance_->attr("declare_var")(var_name.c_str(), var_type.c_str());
  }
}

void StlDiscreteTimeSpecificationWrapper::SetSamplingPeriod(int sampling_period, std::string unit, double tolerance) {
  if (pInstance_) {
    pInstance_->attr("set_sampling_period")(sampling_period, unit.c_str(), tolerance);
  }
}

void StlDiscreteTimeSpecificationWrapper::SetStringAttribute(std::string attr_name, std::string attr_value) {
  if (pInstance_) {
    pInstance_->attr(attr_name.c_str()) = attr_value;
  }
}

void StlDiscreteTimeSpecificationWrapper::CallMethodWithNoArg(std::string method_name) {
  if (pInstance_) {
    pInstance_->attr(method_name.c_str())();
  }
}

double StlDiscreteTimeSpecificationWrapper::UpdateStlSpec(double stl_spec_timestep, 
                                                            std::vector<std::pair<std::string, double>> dataset) const {
  double pyRobustness = std::numeric_limits<double>::infinity();

  if (pInstance_) {
    py::list datasetList;
    for (const auto& pair : dataset) {
      datasetList.append(py::make_tuple(pair.first, pair.second));
    }
    py::object result = pInstance_->attr("update")(stl_spec_timestep, datasetList);
    pyRobustness = result.cast<double>();
    // std::cout << "pyRobustness: " << pyRobustness << std::endl;
  }

  return pyRobustness;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
