// Copyright (c) 2024 fortiss GmbH
//
// Authors: Esra Acar-Celik and Xiangzhong Liu
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "evaluator_stl.hpp"

#include <cmath>
#include <limits>

#include "bark/world/observed_world.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace world {
namespace evaluation {

#ifdef LTL_RULES

EvaluatorSTL::EvaluatorSTL(bark::world::objects::AgentId agent_id,
                           const std::string& ltl_formula_str,
                           const LabelFunctions& label_functions,
                           const bool eval_return_without_robustness)
    : EvaluatorLTL(agent_id, ltl_formula_str, label_functions),
      eval_return_without_robustness_(eval_return_without_robustness) {
}

double EvaluatorSTL::GetRuleViolationPenalty() {
  double robutness_val = ComputeRobustness();
  // std::cout << "EvaluatorSTL Robustness: " << robutness_val << std::endl;
  double penalty = EvaluatorLTL::GetRuleViolationPenalty() - robutness_val;

  // std::cout << "GetRuleViolationPenalty(): " << penalty << std::endl; 
  return penalty;
}

double EvaluatorSTL::ComputeRobustness() {
  double robustness = std::numeric_limits<double>::infinity();

  for (auto le : label_functions_) {
     bark::world::evaluation::BaseQuantizedLabelFunction* quantizedLabelFunction = dynamic_cast<bark::world::evaluation::BaseQuantizedLabelFunction*>(le.get());

    if (quantizedLabelFunction) {
      // std::cout << "quantizedLabelFunction Robustness: " << quantizedLabelFunction->GetCurrentRobustness() << std::endl;
      robustness = std::min(robustness, quantizedLabelFunction->GetCurrentRobustness());
    }
  }

  if (std::isinf(robustness) || std::isnan(robustness)) {
    robustness = 0.0;
  }

  return robustness;
}

EvaluationReturn EvaluatorSTL::Evaluate(
      const world::ObservedWorld& observed_world) {
  EvaluationReturn ltl_result = EvaluatorLTL::Evaluate(observed_world);
  double double_value = std::numeric_limits<double>::infinity();

  if (double* d = boost::get<double>(&ltl_result)) {        
    double double_value = *d;
    // std::cout << "EvaluatorLTL return in STL: " << double_value << std::endl;
  } else {
    std::cout << "EvaluatorLTL return in STL DOES NOT hold a double value." << std::endl;
  }

  if (eval_return_without_robustness_) {
    // std::cout << "EvaluatorSTL evaluation result: " << double_value << std::endl;
    return ltl_result;
  } else {      
    double robutness_val = ComputeRobustness();
    // std::cout << "EvaluatorSTL Robustness: " << robutness_val << std::endl;

    std::string stl_result = std::to_string(double_value) + std::to_string(robutness_val);

    // std::cout << "EvaluatorSTL evaluation result: " << stl_result << std::endl;
    return stl_result;
  }
}

#else
EvaluatorSTL::EvaluatorSTL(){};
#endif

}  // namespace evaluation
}  // namespace world
}  // namespace bark
