// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_LOCALIZATION__SCAN_HPP_
#define IKE_LOCALIZATION__SCAN_HPP_

#include <iostream>
#include <vector>

struct Scan
{
  float angle_min;
  float angle_max;
  float angle_increment;
  float range_min;
  float range_max;
  std::vector<float> ranges;
};

#endif  // IKE_LOCALIZATION__SCAN_HPP_