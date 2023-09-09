# ike_costmap_2d

## Package overview
2次元コストマップ（障害物情報の管理）のROS 2 C++実装です。

2次元地図[`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)、2次元センサ情報[`sensor_msgs/LaserScan`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)から静的レイヤー[`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)、障害物の範囲を広げたレイヤー[`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)、動的障害物の情報を含んだレイヤー[`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)を各レイヤーを統合した2次元コストマップ[`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)を出力します。

## Input / Output

### Input

| **Name（Topic）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/scan`          | `sensor_msgs::msg::LaserScan`                  |      2次元センサ情報    | 

| **Name（Service）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/get_map`          | `ike_nav_msgs::srv::GetMap`                  |      2次元地図    | 

### Output

| **Name（Topic）**        | **Type**                                 | **Description**                                      | 
| -------------------- | ------------------------------------ | ------------------------------------------------ | 
| `/static_layer`          | `nav_msgs::msg::OccupancyGrid`                  | 静的レイヤー         | 
| `/inflation_layer`          | `nav_msgs::msg::OccupancyGrid`                  | 障害物の範囲を広げたレイヤー         | 
| `/obstacle_layer`          | `nav_msgs::msg::OccupancyGrid`                  | 動的障害物の情報を含んだレイヤー         | 
| `/costmap_2d`          | `nav_msgs::msg::OccupancyGrid`                  | 各レイヤーを統合した2次元コストマップ         | 

| **Name（Service）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/get_costmap_2d`          | `nav_msgs::msg::OccupancyGrid`                  |     2次元コストマップ     | 

### [Parameters](../ike_nav_parameters/config/ike_costmap_2d_parameter.yaml)

| **Name（Parameter）**   | **Type**        | **Description**            | 
| ------------------- | ----------- | ---------------------- | 
| `inflation_layer/inflation_radius`           | `double` | どのくらい障害物の範囲を広げるか           | 
| `obstacle_layer/inflation_radius`           | `double` | どのくらい障害物の範囲を広げるか           | 
| `obstacle_layer/obstacle_range`           | `double` | どのくらいの距離の障害物を観測した時に動的障害物であると判断するか           | 
| `publish_costmap_2d_hz`           | `double` |      2次元コストマップのパブリッシュ周期      | 

## LICENSE

Apache License, Version 2.0に基づいています。
