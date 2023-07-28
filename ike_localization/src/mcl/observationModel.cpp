// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "ike_localization/mcl/observationModel.hpp"

namespace mcl
{
ObservationModel::ObservationModel(
  std::shared_ptr<mcl::LikelihoodField> likelihood_field, float angle_min, float angle_max,
  float angle_increment, float range_min, float range_max)
: likelihood_field_(std::move(likelihood_field)), marginal_likelihood_(0.)
{
  std::cout << "Run ObservationModel::ObservationModel."
            << "\n";

  initScan(angle_min, angle_max, angle_increment, range_min, range_max);

  std::cout << "Done ObservationModel::ObservationModel."
            << "\n";
};
ObservationModel::~ObservationModel(){};

void ObservationModel::initScan(
  float angle_min, float angle_max, float angle_increment, float range_min, float range_max)
{
  std::cout << "Run ObservationModel::initScan."
            << "\n";

  scan_.angle_min = angle_min;
  scan_.angle_max = angle_max;
  scan_.angle_increment = angle_increment;
  scan_.range_min = range_min;
  scan_.range_max = range_max;

  std::cout << "Done ObservationModel::initScan."
            << "\n";
}

void ObservationModel::setScan(std::vector<float> & scan_data)
{
  std::cout << "Run ObservationModel::setScan."
            << "\n";

  scan_.ranges.resize(scan_data.size());
  std::copy(std::begin(scan_data), std::end(scan_data), std::begin(scan_.ranges));

  std::cout << "Done ObservationModel::setScan."
            << "\n";
}

void ObservationModel::update(std::vector<Particle> & particles, std::vector<float> scan_data)
{
  std::cout << "Run ObservationModel::update."
            << "\n";

  setScan(scan_data);

  particles_scan_match_point_.clear();
  double sum_score = 0.;
  for (auto & p : particles) {
    auto particle_weight = calculateParticleWeight(p);

    p.weight *= particle_weight;
    sum_score += particle_weight;
  }
  marginal_likelihood_ = sum_score / (particles.size() * scan_.ranges.size());

  std::cout << "Done ObservationModel::update."
            << "\n";
}

double ObservationModel::calculateParticleWeight(const Particle p)
{
  // std::cout << "Run ObservationModel::calculateParticleWeight."
  //           << "\n";

  std::vector<double> hit_xy;
  double particle_weight = 0.;
  double scan_angle_increment = scan_.angle_increment;
  for (auto scan_range : scan_.ranges) {
    ++scan_angle_increment;
    if (scan_range == INFINITY || scan_range == NAN) continue;

    hit_xy.clear();
    hit_xy.push_back(
      p.pose.position.x + scan_range * cos(p.pose.euler.yaw + getRadian(scan_angle_increment)));
    hit_xy.push_back(
      p.pose.position.y + scan_range * sin(p.pose.euler.yaw + getRadian(scan_angle_increment)));
    particles_scan_match_point_.push_back(hit_xy);
    particle_weight += getProbFromLikelihoodMap(hit_xy.at(0), hit_xy.at(1));
  }

  // std::cout << "Done ObservationModel::calculateParticleWeight."
  //           << "\n";
  return particle_weight;
}

double ObservationModel::getProbFromLikelihoodMap(double x, double y)
{
  // std::cout << "Done ObservationModel::getProbFromLikelihoodMap."
  //           << "\n";
  return likelihood_field_->data_
           [likelihood_field_->width_ *
              (likelihood_field_->height_ -
               (floor(std::negate<double>()(y) / likelihood_field_->resolution_) +
                (likelihood_field_->width_ * 0.5) -
                (fabs(
                  fabs(likelihood_field_->origin_y_ / likelihood_field_->resolution_) -
                  fabs(likelihood_field_->width_) * 0.5))) -
               1) +
            (likelihood_field_->height_ * 0.5) +
            (fabs(
              fabs(likelihood_field_->origin_x_ / likelihood_field_->resolution_) -
              fabs(likelihood_field_->height_) * 0.5)) +
            floor(x / likelihood_field_->resolution_)] *
         0.01;
}

}  // namespace mcl