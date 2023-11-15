// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_LOCALIZATION__OBSERVATIONMODEL_HPP_
#define IKE_LOCALIZATION__OBSERVATIONMODEL_HPP_

#include "ike_localization/mcl/likelihoodField.hpp"
#include "ike_localization/mcl/particle.hpp"
#include "ike_localization/mcl/scan.hpp"

#include <cmath>
#include <memory>

namespace mcl
{
class ObservationModel
{
public:
  ObservationModel(
    std::shared_ptr<mcl::LikelihoodField> likelihood_field, float angle_min, float angle_max,
    float angle_increment, float range_min, float range_max,
    bool publish_particles_scan_match_point);
  ~ObservationModel();

  void initScan(
    float angle_min, float angle_max, float angle_increment, float range_min,
    float range_max);  // スキャンに関するパラメータをセットする
  void setScan(std::vector<float> & scan_data);  // ROS 2のスキャンをMClのスキャンとしてセットする

  void update(std::vector<Particle> & particles, std::vector<float> scan_data);  // 観測モデルの更新
  double calculateParticleWeight(const Particle p);  // パーティクルの重みを計算する
  double getProbFromLikelihoodMap(double x, double y);  // 尤度場から確率を取得する

  inline double getRadian(double degree) { return degree * M_PI / 180; }  // ラジアンに変換する

  std::shared_ptr<mcl::LikelihoodField> likelihood_field_;
  Scan scan_;

  float marginal_likelihood_;
  std::vector<std::vector<double>> particles_scan_match_point_;
  bool publish_particles_scan_match_point_;
};
}  // namespace mcl

#endif