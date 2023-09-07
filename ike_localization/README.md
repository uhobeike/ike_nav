# ike_localization

## Package overview
Monte Carlo localization[^1]のROS 2実装です。

2次元地図[`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)、
2次元センサ情報[`sensor_msgs/LaserScan`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)、
移動ロボットのオドメトリ情報[`tf2_msgs/TFMessage`](http://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html)から、2次元地図上での自己位置[`geometry_msgs/PoseStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)を推定します。

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
| `/particle_cloud`      | `nav2_msgs::msg::ParticleCloud`        | パーティクル群群                                   | 
| `/mcl_pose`            | `geometry_msgs::msg::PoseStamped`      |  パーティクル全体の平均姿勢                       | 
| `/marginal_likelihood` | `std_msgs::msg::Float32`               | 周辺尤度                                         | 
| `/scan_match_point`    | `visualization_msgs::msg::MarkerArray` | 各パーティクルのスキャンと尤度場のマッチポイント | 

### [Parameters](../ike_nav_parameters/config/ike_localization_parameter.yaml)

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

### Reference

* 動作モデル
  * https://github.com/ros-planning/navigation2/blob/ef4de1527997c3bd813afe0c6296ff65e05700e0/nav2_amcl/src/motion_model/differential_motion_model.cpp#L57-L61

  * https://github.com/ros-planning/navigation2/blob/ef4de1527997c3bd813afe0c6296ff65e05700e0/nav2_amcl/src/motion_model/differential_motion_model.cpp#L75-L80 

* マップ座標系からオドメトリ座標系への変換
  * https://github.com/ros-planning/navigation2/blob/ef4de1527997c3bd813afe0c6296ff65e05700e0/nav2_amcl/src/amcl_node.cpp#L975-L1016

* [ros-planning/navigation2/nav2_amcl](https://github.com/ros-planning/navigation2/tree/main/nav2_amcl)
* 『詳解 確率ロボティクス ― Pythonによる基礎アルゴリズムの実装 ―』講談社〈KS理工学専門書〉、2019年、ISBN 978-406-51-7006-9
* 『確率ロボティクス』 上田隆一 訳、毎日コミュニケーションズ〈MYCOM ROBOT books〉、2007年、ISBN 9784839924010。（プレミアムブック版 ISBN 978-483-99-5298-3）

## LICENSE

動作モデルとtf周りで[navigation2/nav2_amcl](https://github.com/ros-planning/navigation2/tree/main/nav2_amcl)
を参考にした箇所があるため[GPL v3.0](./LICENSE)ライセンスとなっています。

### Citation

[^1]: Jens-Steffen Gutmann and Dieter Fox: ``An Experimental Comparison of Localization Methods Continued,'' Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp.~454-459, 2002.