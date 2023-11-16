// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "ike_localization/mcl/resampling.hpp"

#include <cstring>
#include <iostream>

namespace mcl
{
Resampling::Resampling(int particle_size) : particle_size_(particle_size){};
Resampling::~Resampling(){};

void Resampling::resampling(std::vector<Particle> & particles)
{
  // std::cerr << "Run Resampling::resampling."
  //           << "\n";

  normalize(particles);
  systematicSampling(particles);

  // std::cerr << "Done Resampling::resampling."
  //           << "\n";
}

// 詳解 確率ロボティクス p.131
void Resampling::systematicSampling(std::vector<Particle> & particles)
{
  std::vector<double> particles_weight_sum;
  double particles_weight_sum_tmp = 0.;
  for (auto p : particles) {
    particles_weight_sum_tmp += p.weight;
    particles_weight_sum.push_back(particles_weight_sum_tmp);
  }

  // clang-format off
  int index = 0;
  double step = 0.;
  std::vector<Particle> new_particles;
  double systematic_sampling_start = calculateSystematicSamplingStart(1. / static_cast<double>(particle_size_));
  double systematic_sampling_step = calculateSystematicSamplingStep();
  while (true) {
    if (new_particles.size() == particle_size_) {
      break;
    }

    if (systematic_sampling_start + step < particles_weight_sum[index]) {
      new_particles.push_back(particles[index]);
      step += systematic_sampling_step;
    } else {
      index++;
    }
  }
  // clang-format on

  std::copy(std::begin(new_particles), std::end(new_particles), std::begin(particles));
}

double Resampling::calculateSystematicSamplingStart(double random_max)
{
  std::uniform_real_distribution<> real_number_generator(0, random_max);

  return real_number_generator(engine_);
}

double Resampling::calculateSystematicSamplingStep()
{
  return 1. / static_cast<double>(particle_size_);
}

void Resampling::normalize(std::vector<Particle> & particles)
{
  double particles_weight_sum = 0;
  for (auto p : particles) particles_weight_sum += p.weight;

  for (auto & p : particles) p.weight /= particles_weight_sum;
}

}  // namespace mcl