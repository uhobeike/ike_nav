// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_LOCALIZATION__MOTIONMODEL_HPP_
#define IKE_LOCALIZATION__MOTIONMODEL_HPP_

#include "ike_localization//mcl/particle.hpp"

#include <random>
#include <vector>

namespace mcl
{
class MotionModel
{
public:
  MotionModel(
    double alpha_trans_trans, double alpha_trans_rotate, double alpha_rotate_trans,
    double alpha_rotate_rotate);
  ~MotionModel();

  void update(
    std::vector<Particle> & particles, double current_pose_yaw, double delta_x, double delta_y,
    double delta_yaw);                                // 動作モデルの更新
  double drawNoise(double sigma);                     // ノイズを一様分布からドロー
  double diffMinAngle(double angle1, double angle2);  // angle1とangle2の最小回転差分を計算
  void getDelta(
    double & delta_x, double & delta_y, double & delta_yaw, double current_x, double past_x,
    double current_y, double past_y, double current_yaw,
    double past_yaw);  // 現在の姿勢と現在の姿勢を比較したときの各差分を計算する
  inline double normalizeAngle(double yaw)
  {
    return atan2(sin(yaw), cos(yaw));
  }  // 回転の正規化を行う（θ ∈ [-π,π)）

  double alpha_trans_trans_, alpha_trans_rotate_, alpha_rotate_trans_,
    alpha_rotate_rotate_;  // 動作モデル用の誤差

  std::random_device seed_gen_;
  std::default_random_engine engine_;
};
}  // namespace mcl

#endif  // IKE_LOCALIZATION__MOTIONMODEL_HPP_