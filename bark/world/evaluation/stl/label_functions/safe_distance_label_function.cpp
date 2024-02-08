// Copyright (c) 2024 fortiss GmbH
//
// Authors: Esra Acar-Celik and Xiangzhong Liu
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <algorithm>

#include "safe_distance_label_function.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/commons/transformation/frenet_state.hpp"
#include "bark/models/dynamic/single_track.hpp"

namespace bark {
namespace world {
namespace evaluation {

using bark::models::dynamic::StateDefinition;
using bark::world::objects::AgentId;
using bark::world::objects::AgentPtr;
using bark::world::FrontRearAgents;

double SafeDistanceQuantizedLabelFunction::robustness_min_ = std::numeric_limits<double>::infinity();
double SafeDistanceQuantizedLabelFunction::robustness_max_ = -std::numeric_limits<double>::infinity();

SafeDistanceQuantizedLabelFunction::SafeDistanceQuantizedLabelFunction(const std::string& label_str, bool to_rear,
                            double delta_ego, double delta_others, double a_e, double a_o,
                            bool consider_crossing_corridors,
                            unsigned int max_agents_for_crossing,
                            bool use_frac_param_from_world,
                            double lateral_difference_threshold,
                            double angle_difference_threshold,
                            bool check_lateral_dist,
                            int signal_sampling_period = 200,
                            bool robustness_normalized = true)
    :SafeDistanceLabelFunction(label_str, to_rear,
                           delta_ego, delta_others, a_e, a_o,
                           consider_crossing_corridors,
                           max_agents_for_crossing,
                           use_frac_param_from_world,
                           lateral_difference_threshold,
                           angle_difference_threshold,
                           check_lateral_dist),    
      robustness_normalized_(robustness_normalized),
      signal_sampling_period_(signal_sampling_period) {
  InitializeSpecs();

  robustness_lon_ = std::numeric_limits<double>::lowest();
  robustness_lat_ = std::numeric_limits<double>::lowest();
  robustness_ = std::numeric_limits<double>::lowest();                        
}

bool SafeDistanceQuantizedLabelFunction::InitializePyObjs() {
  stl_spec_lat_wrapper_.InitializePyObj();
  stl_spec_lon_wrapper_.InitializePyObj();
}

void SafeDistanceQuantizedLabelFunction::InitializeSpecs() {
  bool success = InitializePyObjs();

  if (!success) {
    std::cerr << "RTAMT Exception: Python objects CANNOT be created" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  this->stl_spec_timestep_ = 0;
  this->stl_spec_lon_checked_ = false;
  this->stl_spec_lat_checked_ = false;

  stl_spec_lon_wrapper_.DeclareVar("dist", "float");
  stl_spec_lon_wrapper_.DeclareVar("safe_dist_0", "float");
  stl_spec_lon_wrapper_.DeclareVar("safe_dist_1", "float");
  stl_spec_lon_wrapper_.DeclareVar("safe_dist_2", "float");
  stl_spec_lon_wrapper_.DeclareVar("safe_dist_3", "float");
  stl_spec_lon_wrapper_.DeclareVar("delta", "float");
  stl_spec_lon_wrapper_.DeclareVar("t_stop_f", "float");
  stl_spec_lon_wrapper_.DeclareVar("t_stop_f_star", "float");
  stl_spec_lon_wrapper_.DeclareVar("a_f", "float");
  stl_spec_lon_wrapper_.DeclareVar("a_r", "float");
  stl_spec_lon_wrapper_.DeclareVar("v_f_star", "float");
  stl_spec_lon_wrapper_.DeclareVar("v_r", "float");
  stl_spec_lon_wrapper_.DeclareVar("t_stop_r", "float");

  stl_spec_lon_wrapper_.SetStringAttribute("unit", "s");
  stl_spec_lon_wrapper_.SetSamplingPeriod(signal_sampling_period_, "ms", 0.1);

  std::string formula_lon = "(dist < 0.0)" \
    " or ((dist > safe_dist_0 or (delta <= t_stop_f and dist > safe_dist_3))" \
    " or ((delta <= t_stop_f and a_f > a_r and v_f_star < v_r and t_stop_r < t_stop_f_star)) and (dist > safe_dist_2))" \
    " or (dist > safe_dist_1)";    

  stl_spec_lon_wrapper_.SetStringAttribute("spec", formula_lon);
  stl_spec_lon_wrapper_.CallMethodWithNoArg("parse");
  stl_spec_lon_wrapper_.CallMethodWithNoArg("pastify");
    
  stl_spec_lat_wrapper_.DeclareVar("dist_lat", "float");
  stl_spec_lat_wrapper_.DeclareVar("lateral_positive", "float");
  stl_spec_lat_wrapper_.DeclareVar("v_1_lat", "float");
  stl_spec_lat_wrapper_.DeclareVar("v_2_lat", "float");
  stl_spec_lat_wrapper_.DeclareVar("min_lat_safe_dist", "float");

  stl_spec_lat_wrapper_.SetStringAttribute("unit", "s");
  stl_spec_lat_wrapper_.SetSamplingPeriod(signal_sampling_period_, "ms", 0.1);

  std::string formula_lat = "dist_lat !== 0.0 and" \
      " ((v_1_lat >= 0.0 and v_2_lat <= 0.0 and dist_lat < 0.0)" \
      " or (v_1_lat <= 0.0 and v_2_lat >= 0.0 and dist_lat > 0.0)" \
      " or (lateral_positive > min_lat_safe_dist))";
  stl_spec_lat_wrapper_.SetStringAttribute("spec", formula_lat);
  stl_spec_lat_wrapper_.CallMethodWithNoArg("parse");
  stl_spec_lat_wrapper_.CallMethodWithNoArg("pastify");

  // std::cout << "Successfully parsed the SD STL formulas" << std::endl;
}

void SafeDistanceQuantizedLabelFunction::ComputeRobustness(bool safe_distance) {
    if (this->stl_spec_lon_checked_) {
        this->robustness_lon_ = NormalizeRobustness(this->robustness_lon_, 
                                                    SafeDistanceQuantizedLabelFunction::feature_range_, 
                                                    SafeDistanceQuantizedLabelFunction::lon_value_range_);
    }

    if (this->stl_spec_lat_checked_) {
        this->robustness_lat_ = NormalizeRobustness(this->robustness_lat_, 
                                                    SafeDistanceQuantizedLabelFunction::feature_range_, 
                                                    SafeDistanceQuantizedLabelFunction::lat_value_range_);
    }

    if (!this->stl_spec_lon_checked_ && !this->stl_spec_lat_checked_) {
        if (safe_distance) {
            if (SafeDistanceQuantizedLabelFunction::robustness_max_ >= 0.0) {
                this->robustness_ = SafeDistanceQuantizedLabelFunction::robustness_max_;
            } else {
                this->robustness_ = std::max(SafeDistanceQuantizedLabelFunction::feature_range_.first, 
                  SafeDistanceQuantizedLabelFunction::feature_range_.second);
            }
        } else {
            if (SafeDistanceQuantizedLabelFunction::robustness_min_ <= 0.0) {
                this->robustness_ = SafeDistanceQuantizedLabelFunction::robustness_min_;
            } else {
                this->robustness_ = std::min(SafeDistanceQuantizedLabelFunction::feature_range_.first, 
                  SafeDistanceQuantizedLabelFunction::feature_range_.second);
            }
        }
    } else if (this->stl_spec_lon_checked_ && this->stl_spec_lat_checked_) {
        if (safe_distance && (this->robustness_lon_ < 0.0 || this->robustness_lat_ < 0.0)) {
            this->robustness_ = std::max(this->robustness_lon_, this->robustness_lat_);
        } else {
            this->robustness_ = std::min(this->robustness_lon_, this->robustness_lat_);
        }
    } else if (this->stl_spec_lon_checked_) {
        this->robustness_ = this->robustness_lon_;
    }

    if (this->robustness_ > SafeDistanceQuantizedLabelFunction::robustness_max_) {
        SafeDistanceQuantizedLabelFunction::robustness_max_ = this->robustness_;
    }

    if (this->robustness_ < SafeDistanceQuantizedLabelFunction::robustness_min_) {
        SafeDistanceQuantizedLabelFunction::robustness_min_ = this->robustness_;
    }
}

double SafeDistanceQuantizedLabelFunction::NormalizeRobustness(double robustness, 
                                                                std::pair<double, double> feature_range, 
                                                                std::pair<double, double> value_range) {
    if (this->robustness_normalized_) {
        double min_value = value_range.first;
        double max_value = value_range.second;
        double min_range = feature_range.first;
        double max_range = feature_range.second;

        double scaled_robustness = ((robustness - min_value) / (max_value - min_value)) * (max_range - min_range) + min_range;
        // std::cout << "ROBUSTNESS NORMALIZED: " << scaled_robustness << std::endl;

        return scaled_robustness;
    } else {
        return robustness;
    }
}

LabelMap SafeDistanceQuantizedLabelFunction::Evaluate(const world::ObservedWorld& observed_world) const {
  this->stl_spec_timestep_ = observed_world.GetWorldTime();
  this->stl_spec_lon_checked_ = false;
  this->stl_spec_lat_checked_ = false;

  LabelMap eval_result = SafeDistanceLabelFunction::Evaluate(observed_world);

  // Compute robustness based on evaluation result
  const_cast<SafeDistanceQuantizedLabelFunction*>(this)->ComputeRobustness(eval_result.begin()->second);

  return eval_result;
}

bool SafeDistanceQuantizedLabelFunction::CheckSafeDistanceLongitudinal(
    const float v_f, const float v_r, const float dist,
    const double a_r,  const double a_f, const double delta) const {
  this->stl_spec_lon_checked_ = true;

  // Calculate v_f_star, t_stop_f_star, t_stop_r, and t_stop_f
  double v_f_star = SafeDistanceLabelFunction::CalcVelFrontStar(v_f, a_f, delta);
  double t_stop_f_star = -v_f_star / a_r;
  double t_stop_r = -v_r / a_r;
  double t_stop_f = -v_f / a_f;

  // Calculate safe distances
  double safe_dist_0 = std::max(0.0, SafeDistanceLabelFunction::CalcSafeDistance0(v_r, a_r, delta));
  double safe_dist_1 = std::max(0.0, SafeDistanceLabelFunction::CalcSafeDistance1(v_r, v_f, a_r, a_f, delta));
  double safe_dist_2 = std::max(0.0, SafeDistanceLabelFunction::CalcSafeDistance2(v_r, v_f, a_r, a_f, delta));
  double safe_dist_3 = std::max(0.0, SafeDistanceLabelFunction::CalcSafeDistance3(v_r, v_f, a_r, a_f, delta));

  // Update STL monitor
  std::vector<std::pair<std::string, double>> dataset;

  dataset.push_back({"dist", dist});
  dataset.push_back({"safe_dist_0", safe_dist_0});
  dataset.push_back({"safe_dist_1", safe_dist_1});
  dataset.push_back({"safe_dist_2", safe_dist_2});
  dataset.push_back({"safe_dist_3", safe_dist_3});
  dataset.push_back({"delta", delta});
  dataset.push_back({"t_stop_f", t_stop_f});
  dataset.push_back({"t_stop_f_star", t_stop_f_star});
  dataset.push_back({"a_f", a_f});
  dataset.push_back({"a_r", a_r});
  dataset.push_back({"v_f_star", v_f_star});
  dataset.push_back({"v_r", v_r});
  dataset.push_back({"t_stop_r", t_stop_r});

  this->robustness_lon_ = stl_spec_lon_wrapper_.UpdateStlSpec(this->stl_spec_timestep_, dataset);

  // Determine safe_distance_lon
  bool safe_distance_lon = this->robustness_lon_ > 0.0;

  if (this->robustness_lon_ == 0.0) {
    safe_distance_lon = SafeDistanceLabelFunction::CheckSafeDistanceLongitudinal(v_f, v_r, dist, a_r, a_f, delta);
  }

  return safe_distance_lon;
}

bool SafeDistanceQuantizedLabelFunction::CheckSafeDistanceLateral(
    const float v_1_lat, const float v_2_lat, const float dist_lat,
    const double a_1_lat,  const double a_2_lat, const double delta1,
    const double delta2) const {
  this->stl_spec_lat_checked_ = true;

  // Make v_1_lat larger than v_2_lat if necessary
  float v_1_lat_non_const = v_1_lat;
  float v_2_lat_non_const = v_2_lat;

  float delta1_non_const = delta1;
  float delta2_non_const = delta2;

  float a_1_lat_non_const = a_1_lat;
  float a_2_lat_non_const = a_2_lat;

  if (v_1_lat < v_2_lat) {
    std::swap(v_1_lat_non_const, v_2_lat_non_const);
    std::swap(delta1_non_const, delta2_non_const);
    std::swap(a_1_lat_non_const, a_2_lat_non_const);
  }

  // Calculate lateral distance positive
  double lateral_positive = fabs(dist_lat);

  // Calculate min_lat_safe_dist
  double min_lat_safe_dist = v_1_lat_non_const*delta1_non_const 
                                        + (v_1_lat_non_const == 0.0 ? 0.0 : v_1_lat_non_const*v_1_lat_non_const / (2 * a_1_lat_non_const)) 
                                        - (v_2_lat_non_const*delta2_non_const 
                                        - (v_2_lat_non_const == 0.0 ? 0.0 : v_2_lat_non_const*v_2_lat_non_const / (2 * a_2_lat_non_const)));;

  // Update STL monitor
  std::vector<std::pair<std::string, double>> dataset;

  dataset.push_back({"dist_lat", dist_lat});
  dataset.push_back({"lateral_positive", lateral_positive});
  dataset.push_back({"v_1_lat", v_1_lat});
  dataset.push_back({"v_2_lat", v_2_lat});
  dataset.push_back({"min_lat_safe_dist", min_lat_safe_dist});

  this->robustness_lat_ = stl_spec_lat_wrapper_.UpdateStlSpec(this->stl_spec_timestep_, dataset);
  
  // Determine safe_distance_lat
  bool safe_distance_lat = this->robustness_lat_ > 0.0;

  if (this->robustness_lat_ == 0.0) {
    safe_distance_lat = SafeDistanceLabelFunction::CheckSafeDistanceLateral(v_1_lat, v_2_lat, dist_lat, a_1_lat, a_2_lat, delta1, delta2);
  }

  return safe_distance_lat;          
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
