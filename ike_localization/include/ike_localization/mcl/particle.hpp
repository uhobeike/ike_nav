// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_LOCALIZATION__PARTICLE_HPP_
#define IKE_LOCALIZATION__PARTICLE_HPP_

struct Euler
{
  double yaw;
};

struct Point
{
  double x;
  double y;
};

struct Pose
{
  Point position;
  Euler euler;
};

struct Particle
{
  Pose pose;

  double weight;
};

#endif  // IKE_LOCALIZATION__PARTICLE_HPP_