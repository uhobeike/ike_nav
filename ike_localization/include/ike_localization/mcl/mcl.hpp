// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_LOCALIZATION__MCL_HPP_
#define IKE_LOCALIZATION__MCL_HPP_

#include "ike_localization/mcl/likelihoodField.hpp"
#include "ike_localization/mcl/motionModel.hpp"
#include "ike_localization/mcl/observationModel.hpp"
#include "ike_localization/mcl/particle.hpp"
#include "ike_localization/mcl/resampling.hpp"

#include <memory>
#include <vector>

namespace mcl
{
class Mcl
{
public:
  Mcl(
    double ini_pose_x, double ini_pose_y, double ini_pose_yaw, double alpha_trans_trans,
    double alpha_trans_rotate, double alpha_rotate_trans, double alpha_rotate_rotate,
    int particle_size, double likelihood_dist, uint32_t map_width, uint32_t map_height,
    double map_resolution, double map_origin_x, double map_origin_y, std::vector<int8_t> map_data,
    float scan_angle_min, float scan_angle_max, float scan_angle_increment, float scan_range_min,
    float scan_range_max, bool publish_particles_scan_match_point);
  ~Mcl();

  void release_pointers();

  std::shared_ptr<LikelihoodField> likelihood_field_;    // 尤度場オブジェクト
  std::shared_ptr<MotionModel> motion_model_;            // 動作モデルオブジェクト
  std::shared_ptr<ObservationModel> observation_model_;  // 観測モデルオブジェクト
  std::shared_ptr<Resampling> resampling_;               // リサンプリングオブジェクト

  void initParticles(
    double ini_pose_x, double ini_pose_y, double ini_pose_yaw,
    int particle_size);                       // パーティクルの初期化をする
  void getMeanParticle(Particle & particle);  // 最尤なパーティクルを渡す
  inline float getMarginalLikelihood()
  {
    return observation_model_->marginal_likelihood_;
  }  // 周辺尤度を渡す
  inline std::vector<std::vector<double>> getParticlesScanMatchPoint()
  {
    return observation_model_->particles_scan_match_point_;
  }  // 各パーティクルのスキャンと尤度場のマッチポイントを渡す

  std::vector<Particle> particles_;
};
}  // namespace mcl

#endif  // IKE_LOCALIZATION__MCL_HPP_
