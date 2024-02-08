// Copyright (c) 2024 fortiss GmbH
//
// Authors: Esra Acar-Celik and Xiangzhong Liu
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_STL_LABELS_SAFE_DISTANCE_QUANTIZED_LABEL_FUNCTION_HPP_
#define BARK_WORLD_EVALUATION_STL_LABELS_SAFE_DISTANCE_QUANTIZED_LABEL_FUNCTION_HPP_

#include <string>
#include <vector>
#include <cstdint>

#include "bark/world/evaluation/ltl/label_functions/safe_distance_label_function.hpp"
#include "bark/world/evaluation/stl/label_functions/base_label_function.hpp"
#include "bark/commons/transformation/frenet_state.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/objects/object.hpp"
#include "bark/world/world.hpp"

#include "stl_discrete_time_specification.hpp"

namespace bark {
namespace world {
namespace evaluation {

using bark::commons::transformation::FrenetPosition;
using bark::world::objects::AgentPtr;
using bark::world::FrontRearAgents;

class SafeDistanceQuantizedLabelFunction : public SafeDistanceLabelFunction, public BaseQuantizedLabelFunction {
 public:
  SafeDistanceQuantizedLabelFunction(const std::string& label_str, bool to_rear,
                            double delta_ego, double delta_others, double a_e, double a_o,
                            bool consider_crossing_corridors,
                            unsigned int max_agents_for_crossing,
                            bool use_frac_param_from_world,
                            double lateral_difference_threshold,
                            double angle_difference_threshold,
                            bool check_lateral_dist,
                            int signal_sampling_period,
                            bool robustness_normalized);  
  virtual bool CheckSafeDistanceLongitudinal(
    const float v_f, const float v_r, const float dist,
    const double a_r,  const double a_f, const double delta) const;

  virtual bool CheckSafeDistanceLateral(
    const float v_f_lat, const float v_r_lat, const float dist_lat,
    const double a_r_lat,  const double a_f_lat, const double delta1,
    const double delta2) const;
    
  bool GetRobustnessNormalized() const { return robustness_normalized_; }
  
  int GetSignalSamplingPeriod() const { return signal_sampling_period_; }

 protected:
  virtual LabelMap Evaluate(const world::ObservedWorld& observed_world) const override;

 private:  
  bool InitializePyObjs();

  void InitializeSpecs();

  void ComputeRobustness(bool safe_distance);

  double NormalizeRobustness(double robustness, std::pair<double, double> feature_range, std::pair<double, double> value_range);

  int signal_sampling_period_;
  mutable double robustness_lat_;
  mutable double robustness_lon_;
  mutable double stl_spec_timestep_;

  bool robustness_normalized_;
  mutable bool stl_spec_lat_checked_;
  mutable bool stl_spec_lon_checked_;
  
  StlDiscreteTimeSpecificationWrapper stl_spec_lat_wrapper_;
  StlDiscreteTimeSpecificationWrapper stl_spec_lon_wrapper_;

  static double robustness_min_;
  static double robustness_max_;

  static constexpr std::pair<double, double> feature_range_ = std::make_pair(-2.0, 2.0);
  static constexpr std::pair<double, double> lat_value_range_ = std::make_pair(-20.0, 20.0);
  static constexpr std::pair<double, double> lon_value_range_ = std::make_pair(-200.0, 200.0);
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_STL_LABELS_SAFE_DISTANCE_QUANTIZED_LABEL_FUNCTION_HPP_
