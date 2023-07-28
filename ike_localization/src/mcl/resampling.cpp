// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "ike_localization/mcl/resampling.hpp"

#include <cstring>
#include <iostream>

namespace mcl
{
Resampling::Resampling(int particle_size) : engine_(seed_gen_()), particle_size_(particle_size){};
Resampling::~Resampling(){};

void Resampling::resampling(std::vector<Particle> & particles)
{
  std::cout << "Run Resampling::resampling."
            << "\n";

  systematicSampling(particles);
  normalize(particles);

  std::cout << "Done Resampling::resampling."
            << "\n";
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
  double systematic_sampling_step =
    calculateSystematicSamplingStep(particles_weight_sum.back() / particles.size());

  uint32_t index = 0;
  std::vector<Particle> new_particles;
  while (new_particles.size() <= particle_size_) {
    if (systematic_sampling_step < particles_weight_sum[index]) {
      if (particles[index].weight < 0.3) {
        ++index;
        continue;
      }
      new_particles.push_back(particles[index]);
      ++systematic_sampling_step;
    } else {
      new_particles.push_back(particles[index]);
      ++index;
    }
  }

  std::copy(std::begin(new_particles), std::end(new_particles), std::begin(particles));
}

double Resampling::calculateSystematicSamplingStep(double particles_weight_median)
{
  std::uniform_real_distribution<> real_number_generator(0, particles_weight_median);

  return real_number_generator(engine_);
}

void Resampling::normalize(std::vector<Particle> & particles)
{
  double particles_weight_sum = 0;
  for (auto p : particles) particles_weight_sum += p.weight;

  for (auto & p : particles) p.weight /= particles_weight_sum;
}

}  // namespace mcl