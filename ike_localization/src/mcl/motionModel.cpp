// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "ike_localization/mcl/motionModel.hpp"

#include <cmath>
#include <iostream>

namespace mcl
{
MotionModel::MotionModel(
  double alpha_trans_trans, double alpha_trans_rotate, double alpha_rotate_trans,
  double alpha_rotate_rotate)
: alpha_trans_trans_(alpha_trans_trans),
  alpha_trans_rotate_(alpha_trans_rotate),
  alpha_rotate_trans_(alpha_rotate_trans),
  alpha_rotate_rotate_(alpha_rotate_rotate),
  engine_(seed_gen_())
{
  std::cout << "Done MotionModel::MotionModel."
            << "\n";
};
MotionModel::~MotionModel(){};

// Probabilistic Robotics p.124
void MotionModel::update(
  std::vector<Particle> & particles, double current_pose_yaw, double delta_x, double delta_y,
  double delta_yaw)
{
  std::cout << "Run MotionModel::update."
            << "\n";

  double delta_rotate_1, delta_rotate_2, delta_trans;
  double delta_rotate_1_hat, delta_rotate_2_hat, delta_trans_hat;
  double delta_rotate_1_noise, delta_rotate_2_noise;

  // https://github.com/ros-planning/navigation2/blob/ef4de1527997c3bd813afe0c6296ff65e05700e0/nav2_amcl/src/motion_model/differential_motion_model.cpp#L57-L61
  if (sqrt(delta_x * delta_x + delta_y * delta_y) < 0.01)
    delta_rotate_1 = 0.;
  else
    delta_rotate_1 = diffMinAngle(atan2(delta_y, delta_x), current_pose_yaw);
  delta_trans = sqrt(delta_x * delta_x + delta_y * delta_y);
  delta_rotate_2 = diffMinAngle(delta_yaw, delta_rotate_1);

  // https://github.com/ros-planning/navigation2/blob/ef4de1527997c3bd813afe0c6296ff65e05700e0/nav2_amcl/src/motion_model/differential_motion_model.cpp#L75-L80
  delta_rotate_1_noise =
    std::min(fabs(diffMinAngle(delta_rotate_1, 0.)), fabs(diffMinAngle(delta_rotate_1, M_PI)));
  delta_rotate_2_noise =
    std::min(fabs(diffMinAngle(delta_rotate_2, 0.)), fabs(diffMinAngle(delta_rotate_2, M_PI)));

  for (auto & p : particles) {
    delta_rotate_1_hat = diffMinAngle(
      delta_rotate_1, drawNoise(sqrt(
                        alpha_trans_trans_ * delta_rotate_1_noise * delta_rotate_1_noise +
                        alpha_trans_rotate_ * delta_trans * delta_trans)));
    delta_trans_hat =
      delta_trans - drawNoise(sqrt(
                      alpha_rotate_trans_ * delta_trans * delta_trans +
                      alpha_rotate_rotate_ * delta_rotate_1_noise * delta_rotate_1_noise +
                      alpha_rotate_rotate_ * delta_rotate_2_noise * delta_rotate_2_noise));
    delta_rotate_2_hat = diffMinAngle(
      delta_rotate_2, drawNoise(sqrt(
                        alpha_trans_trans_ * delta_rotate_2_noise * delta_rotate_2_noise +
                        alpha_trans_rotate_ * delta_trans * delta_trans)));

    p.pose.position.x += delta_trans_hat * cos(p.pose.euler.yaw + delta_rotate_1_hat);
    p.pose.position.y += delta_trans_hat * sin(p.pose.euler.yaw + delta_rotate_1_hat);
    p.pose.euler.yaw = normalizeAngle(p.pose.euler.yaw) + delta_rotate_1_hat + delta_rotate_2_hat;
  }

  std::cout << "Done MotionModel::update."
            << "\n";
}

double MotionModel::drawNoise(double sigma)
{
  std::normal_distribution<> dist(0, sigma);

  return dist(engine_);
}

double MotionModel::diffMinAngle(double angle1, double angle2)
{
  angle1 = normalizeAngle(angle1);
  angle2 = normalizeAngle(angle2);
  double diff_angle_1 = angle1 - angle2;
  double diff_angle_2 = 2 * M_PI - fabs(diff_angle_1);

  if (diff_angle_1 > 0.) diff_angle_2 *= -1.;

  return (fabs(diff_angle_1) < fabs(diff_angle_2)) ? diff_angle_1 : diff_angle_2;
}

void MotionModel::getDelta(
  double & delta_x, double & delta_y, double & delta_yaw, double current_x, double past_x,
  double current_y, double past_y, double current_yaw, double past_yaw)
{
  delta_x = current_x - past_x;
  delta_y = current_y - past_y;
  delta_yaw = diffMinAngle(current_yaw, past_yaw);
}

}  // namespace mcl