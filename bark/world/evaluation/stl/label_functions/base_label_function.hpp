// Copyright (c) 2024 fortiss GmbH
//
// Authors: Esra Acar-Celik and Xiangzhong Liu
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_STL_LABELS_BASE_LABEL_FUNCTION_HPP_
#define BARK_WORLD_EVALUATION_STL_LABELS_BASE_LABEL_FUNCTION_HPP_

#include <limits>

namespace bark {
namespace world {
namespace evaluation {

class BaseQuantizedLabelFunction {
public:
    BaseQuantizedLabelFunction(float robustness = -std::numeric_limits<float>::infinity()) 
        : robustness_(robustness) {}

  double GetCurrentRobustness() const {
    return robustness_;
  }
protected:
    double robustness_;        
};
}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_STL_LABELS_BASE_LABEL_FUNCTION_HPP_
