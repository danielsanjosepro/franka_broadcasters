#pragma once
// Based in
// https://github.com/marcocognetti/FrankaEmikaPandaDynModel/tree/master

#include <Eigen/Core>

#define NUMBER_OF_JOINTS 7

namespace Eigen {
typedef Eigen::Matrix<double, NUMBER_OF_JOINTS, NUMBER_OF_JOINTS> Matrix7d;
typedef Eigen::Matrix<double, NUMBER_OF_JOINTS, 1> Vector7d;
} // namespace Eigen

using namespace std;

const double FI_11 = 0.54615;
const double FI_12 = 0.87224;
const double FI_13 = 0.64068;
const double FI_14 = 1.2794;
const double FI_15 = 0.83904;
const double FI_16 = 0.30301;
const double FI_17 = 0.56489;

const double FI_21 = 5.1181;
const double FI_22 = 9.0657;
const double FI_23 = 10.136;
const double FI_24 = 5.5903;
const double FI_25 = 8.3469;
const double FI_26 = 17.133;
const double FI_27 = 10.336;

const double FI_31 = 0.039533;
const double FI_32 = 0.025882;
const double FI_33 = -0.04607;
const double FI_34 = 0.036194;
const double FI_35 = 0.026226;
const double FI_36 = -0.021047;
const double FI_37 = 0.0035526;

const double TAU_F_CONST_1 = FI_11 / (1 + exp(-FI_21 * FI_31));
const double TAU_F_CONST_2 = FI_12 / (1 + exp(-FI_22 * FI_32));
const double TAU_F_CONST_3 = FI_13 / (1 + exp(-FI_23 * FI_33));
const double TAU_F_CONST_4 = FI_14 / (1 + exp(-FI_24 * FI_34));
const double TAU_F_CONST_5 = FI_15 / (1 + exp(-FI_25 * FI_35));
const double TAU_F_CONST_6 = FI_16 / (1 + exp(-FI_26 * FI_36));
const double TAU_F_CONST_7 = FI_17 / (1 + exp(-FI_27 * FI_37));

inline Eigen::Vector7d get_friction(const Eigen::Vector7d &dq) {
  static Eigen::Vector7d tau_f = Eigen::Vector7d::Zero();

  tau_f(0) = FI_11 / (1 + exp(-FI_21 * (dq(0) + FI_31))) - TAU_F_CONST_1;
  tau_f(1) = FI_12 / (1 + exp(-FI_22 * (dq(1) + FI_32))) - TAU_F_CONST_2;
  tau_f(2) = FI_13 / (1 + exp(-FI_23 * (dq(2) + FI_33))) - TAU_F_CONST_3;
  tau_f(3) = FI_14 / (1 + exp(-FI_24 * (dq(3) + FI_34))) - TAU_F_CONST_4;
  tau_f(4) = FI_15 / (1 + exp(-FI_25 * (dq(4) + FI_35))) - TAU_F_CONST_5;
  tau_f(5) = FI_16 / (1 + exp(-FI_26 * (dq(5) + FI_36))) - TAU_F_CONST_6;
  tau_f(6) = FI_17 / (1 + exp(-FI_27 * (dq(6) + FI_37))) - TAU_F_CONST_7;

  return tau_f;
}
