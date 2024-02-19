// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_PYTHON_WRAPPER_WORLD_LTL_HPP_
#define BARK_PYTHON_WRAPPER_WORLD_LTL_HPP_

#include "bark/python_wrapper/common.hpp"
#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/safe_distance_label_function.hpp"
#include "bark/world/evaluation/ltl/evaluator_ltl.hpp"

namespace py = pybind11;
using namespace bark::world::evaluation;
using namespace bark::world;

class PyBaseLabelFunction : public BaseLabelFunction {
 public:
  using BaseLabelFunction::BaseLabelFunction;

  LabelMap Evaluate(const ObservedWorld& observed_world) const override {
    PYBIND11_OVERLOAD_PURE(LabelMap, BaseLabelFunction, Evaluate,
                           observed_world);
  }
};

class PySafeDistanceLabelFunction : public SafeDistanceLabelFunction { 
 public:
    /* Inherit the constructors */
    using SafeDistanceLabelFunction::SafeDistanceLabelFunction;

    using SafeDistanceLabelFunction::CalcVelFrontStar; 
    using SafeDistanceLabelFunction::CalcSafeDistance0; 
    using SafeDistanceLabelFunction::CalcSafeDistance1; 
    using SafeDistanceLabelFunction::CalcSafeDistance2; 
    using SafeDistanceLabelFunction::CalcSafeDistance3; 

    bool CheckSafeDistanceLateral(const float v_1_lat, const float v_2_lat, const float dist_lat,
                                  const double a_1_lat,  const double a_2_lat, const double delta1,
                                  const double delta2) const override {
        PYBIND11_OVERLOAD(
            bool,
            SafeDistanceLabelFunction,     
            CheckSafeDistanceLateral,       
            v_1_lat, v_2_lat, dist_lat, a_1_lat,  a_2_lat, delta1, delta2
        );
    }

    bool CheckSafeDistanceLongitudinal(const float v_f, const float v_r, const float dist,
                                       const double a_r,  const double a_f, const double delta) const override {
        PYBIND11_OVERLOAD(
            bool,
            SafeDistanceLabelFunction,     
            CheckSafeDistanceLongitudinal,       
            v_f, v_r, dist, a_r, a_f, delta
        );
    }

    LabelMap Evaluate(const ObservedWorld& observed_world) const override {
        PYBIND11_OVERLOAD(
            LabelMap,
            SafeDistanceLabelFunction,     
            Evaluate,       
            observed_world
        );
    }

};

class PyEvaluatorLTL : public EvaluatorLTL { 
 public:
    using EvaluatorLTL::Evaluate;     
    using EvaluatorLTL::EvaluatorLTL;
    using EvaluatorLTL::GetRuleViolationPenalty;

    EvaluationReturn Evaluate(const ObservedWorld& observed_world) override {
        PYBIND11_OVERLOAD(
            EvaluationReturn,
            EvaluatorLTL,     
            Evaluate,       
            observed_world
        );
    }

    double GetRuleViolationPenalty() override {
        PYBIND11_OVERLOAD(
          double,
          EvaluatorLTL,
          GetRuleViolationPenalty
    );
}
};

class SafeDistanceQuantizedLabelFunctionWrapper : public SafeDistanceLabelFunction {
public:
  SafeDistanceQuantizedLabelFunctionWrapper(const std::string& label_str, bool to_rear, double delta_ego, double delta_others, 
                                            double a_e, double a_o, bool consider_crossing_corridors, int max_agents_for_crossing, 
                                            bool use_frac_param_from_world, double lateral_difference_threshold, 
                                            double angle_difference_threshold, bool check_lateral_dist, 
                                            double signal_sampling_period, bool robustness_normalized)
    : SafeDistanceLabelFunction(label_str, to_rear, delta_ego, delta_others, 
                              a_e, a_o, consider_crossing_corridors, max_agents_for_crossing, 
                              use_frac_param_from_world, lateral_difference_threshold, 
                              angle_difference_threshold, check_lateral_dist) {
    sdInstance = py::module::import("bark_ml.evaluators.stl.safe_distance_label_function")
                            .attr("SafeDistanceQuantizedLabelFunction")(label_str.c_str(), to_rear, delta_ego, delta_others, 
                            a_e, a_o, consider_crossing_corridors, max_agents_for_crossing, 
                            use_frac_param_from_world, lateral_difference_threshold, 
                            angle_difference_threshold, check_lateral_dist,
                            signal_sampling_period, robustness_normalized);
  };

  LabelMap Evaluate(const bark::world::ObservedWorld& observed_world) const {      
    py::object result = sdInstance.attr("Evaluate")(observed_world);
    return pyObjectToLabelMap(result);
  };

  LabelMap pyObjectToLabelMap(const py::object &obj) const {
    LabelMap labelMap;
    py::dict pyDict = py::cast<py::dict>(obj);

    for (auto item : pyDict) {      
      Label key = py::cast<Label>(item.first);
      bool value = py::cast<bool>(item.second);        
      labelMap[{key}] = value;
    }

    return labelMap;
  };

  double GetSignalSamplingPeriod() const { 
    double signal_sampling_period = sdInstance.attr("GetSignalSamplingPeriod")().cast<double>();
    return signal_sampling_period;
  };

  bool GetRobustnessNormalized() const { 
    bool robustness_normalized = sdInstance.attr("GetRobustnessNormalized")().cast<bool>();
    return robustness_normalized;
  };

  double GetCurrentRobustness() {
    double robustness = sdInstance.attr("GetCurrentRobustness")().cast<double>();
    return robustness;
  };

private:
  py::object sdInstance;  
};

class EvaluatorSTLWrapper : public EvaluatorLTL {
public:
  EvaluatorSTLWrapper(bark::world::objects::AgentId agent_id,
               const std::string& ltl_formula_str,
               const LabelFunctions& label_functions,
               bool eval_return_without_robustness) 
      : EvaluatorLTL(agent_id, ltl_formula_str, label_functions) {
    stlInstance = py::module::import("bark_ml.evaluators.stl.evaluator_stl").attr("EvaluatorSTL")
                                (agent_id, ltl_formula_str.c_str(), label_functions, eval_return_without_robustness);
  };

  EvaluationReturn Evaluate(const bark::world::ObservedWorld& observed_world) {
    py::object result = stlInstance.attr("Evaluate")(observed_world);
    return pyObjectToEvaluationReturn(result);    
  };

  EvaluationReturn pyObjectToEvaluationReturn(const py::object &obj) {
    if (py::isinstance<py::int_>(obj)) {
        return py::cast<int>(obj);
    } else if (py::isinstance<py::float_>(obj)) {
        return py::cast<double>(obj);
    } else if (py::isinstance<py::bool_>(obj)) {
        return py::cast<bool>(obj);
    } else if (py::isinstance<py::str>(obj)) {
        return py::cast<std::string>(obj);
    } else if (py::isinstance<py::none>(obj)) {
        return std::optional<bool>();
    } else {
        throw std::runtime_error("Unsupported type");
    }
  }

  double GetRuleViolationPenalty() {
    double rule_violation_penalty = stlInstance.attr("GetRuleViolationPenalty")().cast<double>();
    return rule_violation_penalty;    
  };       

private:
  py::object stlInstance;  
};

void python_ltl(py::module m);

#endif  // BARK_PYTHON_WRAPPER_WORLD_LTL_HPP_
