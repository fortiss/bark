// Copyright (c) 2024 fortiss GmbH
//
// Authors: Esra Acar-Celik and Xiangzhong Liu
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_STL_EVALUATOR_STL_HPP_
#define BARK_WORLD_EVALUATION_STL_EVALUATOR_STL_HPP_

#include <set>
#include <string>
#include <vector>

#include "bark/world/objects/agent.hpp"
#include "bark/world/world.hpp"
#ifdef LTL_RULES
#include "ltl/rule_state.h"
#endif

#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"
#include "bark/world/evaluation/stl/label_functions/base_label_function.hpp"
#include "bark/world/evaluation/ltl/evaluator_ltl.hpp"

namespace bark {
namespace world {
namespace evaluation {

#ifdef LTL_RULES
using ltl::RuleMonitor;
using ltl::RuleState;
#endif

using objects::AgentId;

class EvaluatorSTL : public EvaluatorLTL {
 public:
#ifdef LTL_RULES
  EvaluatorSTL(bark::world::objects::AgentId agent_id,
               const std::string& ltl_formula_str,
               const LabelFunctions& label_functions,
               const bool eval_return_without_robustness);

 protected:
  EvaluationReturn Evaluate(
      const world::ObservedWorld& observed_world) override;
  double GetRuleViolationPenalty() override; 

 private:
    double ComputeRobustness();

    bool eval_return_without_robustness_;    
#else
  EvaluatorSTL();
#endif
};
}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_STL_EVALUATOR_STL_HPP_