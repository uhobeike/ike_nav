# ike_localization

<div align="center">
<img src="https://user-images.githubusercontent.com/40545422/218437382-8091b9dc-05bf-4410-8071-5c27bc8c9b97.gif" width="1000">
</div>

## Overview
Monte Carlo localizationのROS 2実装です。

このパッケージには、以下のような**特徴**があります。
* MCl実装とROS 2実装を分割
* シンプルなMCl実装（可読性があるかは。。。）
* 尤度場の可視化
* 各パーティクルのスキャンと尤度場のマッチポイントの可視化
* 日本語のコメントアウトが多め（ヘッダーファイルに説明が書かれている）

## Input / Output

### Input

| **Name（Topic）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/map`          | `nav_msgs::msg::OccupancyGrid`                  | map_serverから受け取る2次元地図         | 
| `/scan`         | `sensor_msgs::msg::LaserScan`                   | ロボットから得られる2次元スキャンデータ | 
| `/initialpose`  | `geometry_msgs::msg::PoseWithCovarianceStamped` | 自己位置推定のための初期値              | 

### Output

| **Name（Topic）**        | **Type**                                 | **Description**                                      | 
| -------------------- | ------------------------------------ | ------------------------------------------------ | 
| `/likelihood_map`      | `nav_msgs::msg::OccupancyGrid`         | 尤度場                                           | 
| `/particle_cloud`      | `nav2_msgs::msg::ParticleCloud`        | パーティクル群                                   | 
| `/mcl_pose`            | `geometry_msgs::msg::PoseStamped`      | 最尤なパーティクルの姿勢                         | 
| `/marginal_likelihood` | `std_msgs::msg::Float32`               | 周辺尤度                                         | 
| `/scan_match_point`    | `visualization_msgs::msg::MarkerArray` | 各パーティクルのスキャンと尤度場のマッチポイント | 

### Parameters

| **Name（Parameter）**   | **Type**        | **Description**            | 
| ------------------- | ----------- | ---------------------- | 
| `map_frame`           | `std::string` | マップ座標系           | 
| `odom_frame`          | `std::string` | オドメトリ座標系       | 
| `robot_frame`         | `std::string` | ロボット座標系         | 
| `particle_size`       | `int`         | パーティクルの最大数   | 
| `alpha_trans_trans`   | `double`      | 直進で生じる直進の誤差 | 
| `alpha_trans_rotate`  | `double`      | 直進で生じる回転の誤差 | 
| `alpha_rotate_trans`  | `double`      | 回転で生じる直線の誤差 | 
| `alpha_rotate_rotate` | `double`      | 回転で生じる回転の誤差 | 
| `likelihood_dist`     | `double`      | 尤度場距離             | 
| `loop_mcl_hz`         | `double`      | MClの周期              |
| `publish_particles_scan_match_point`         | `bool`        | 各パーティクルのスキャンと尤度場のマッチポイントの可視化（可視化すると、見応えがあるが非常に重い処理）              |

## LICENSE

動作モデルとtf周りで[navigation2/nav2_amcl](https://github.com/ros-planning/navigation2/tree/main/nav2_amcl)
を参考にした箇所があるため[GPL v3.0](./LICENSE)ライセンスとなっています。

### 参考箇所

* 動作モデル
  * https://github.com/ros-planning/navigation2/blob/ef4de1527997c3bd813afe0c6296ff65e05700e0/nav2_amcl/src/motion_model/differential_motion_model.cpp#L57-L61

  * https://github.com/ros-planning/navigation2/blob/ef4de1527997c3bd813afe0c6296ff65e05700e0/nav2_amcl/src/motion_model/differential_motion_model.cpp#L75-L80 

* マップ座標系からオドメトリ座標系への変換
  * https://github.com/ros-planning/navigation2/blob/ef4de1527997c3bd813afe0c6296ff65e05700e0/nav2_amcl/src/amcl_node.cpp#L975-L1016

## Reference

* [ros-planning/navigation2/nav2_amcl](https://github.com/ros-planning/navigation2/tree/main/nav2_amcl)
* 『詳解 確率ロボティクス ― Pythonによる基礎アルゴリズムの実装 ―』講談社〈KS理工学専門書〉、2019年、ISBN 978-406-51-7006-9
* 『確率ロボティクス』 上田隆一 訳、毎日コミュニケーションズ〈MYCOM ROBOT books〉、2007年、ISBN 9784839924010。（プレミアムブック版 ISBN 978-483-99-5298-3）
