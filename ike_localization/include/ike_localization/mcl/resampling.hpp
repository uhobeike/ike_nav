// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_LOCALIZATION__RESAMPLING_HPP_
#define IKE_LOCALIZATION__RESAMPLING_HPP_

#include "ike_localization/mcl/particle.hpp"

#include <random>
#include <vector>

namespace mcl
{
class Resampling
{
public:
  Resampling(int particle_size_);
  ~Resampling();

  void resampling(std::vector<Particle> & particles);  // リサンプリングをする

  void systematicSampling(std::vector<Particle> & particles);  // 系統サンプリングをする
  double calculateSystematicSamplingStart(
    double random_max);  // 系統サンプリング用のランダムなstartを計算する
  double calculateSystematicSamplingStep();  // 系統サンプリング用のstepを計算する
  void normalize(std::vector<Particle> & particles);  // パーティクルの重みを正規化をする

  std::random_device seed_gen_;
  std::mt19937 engine_;

  long unsigned int particle_size_;
};
}  // namespace mcl

#endif