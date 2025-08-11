#pragma once
// Based in
// https://github.com/marcocognetti/FrankaEmikaPandaDynModel/tree/master

#include <Eigen/Core>

#define DELTA_TAU_MAX 1.0

inline Eigen::Matrix<double, 7, 1>
saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
                   const Eigen::Matrix<double, 7, 1>
                       &tau_J_d) { // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] +
        std::max(std::min(difference, DELTA_TAU_MAX), -DELTA_TAU_MAX);
  }
  return tau_d_saturated;
}
