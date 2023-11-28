// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_LOCALIZATION__LIKELIHOODFIELD_HPP_
#define IKE_LOCALIZATION__LIKELIHOODFIELD_HPP_

#include <iostream>
#include <vector>

namespace mcl
{
class LikelihoodField
{
public:
  LikelihoodField(
    double likelihood_dist, uint32_t width, uint32_t height, double resolution, double origin_x,
    double origin_y, std::vector<int8_t> data);
  ~LikelihoodField();

  void createLikelihoodField();                              // 尤度場を作成する
  void calculateLikelihood(uint32_t map_x, uint32_t map_y);  // 尤度場作成のための計算をする
  double calculateProb(
    double stochastic_variable, double likelihood_dist);  // 尤度場作成に必要な確率を求める
  double normalizePdf(double max_pdf, double pdf);  // 足して1になるように確率を修正する

  void getLikelihoodField(std::vector<int8_t> & data);  // 尤度場を渡す

  double likelihood_dist_;    // 尤度場の距離
  uint32_t width_;            // 受け取ったマップのwidth
  uint32_t height_;           // 受け取ったマップのheight
  double resolution_;         // 受け取ったマップの解像度
  double origin_x_;           // 受け取ったマップの原点x
  double origin_y_;           // 受け取ったマップの原点y
  std::vector<double> data_;  // 受け取ったマップの各画素の情報
};
}  // namespace mcl

#endif  // IKE_LOCALIZATION__LIKELIHOODFIELD_HPP_