# ike_controller

## Package overview
Model Predictive ControlのROS 2実装です。

ある経路[`nav_msgs/Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)に対して追従する速度[`geometry_msgs/Twist`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html)を、
差動二輪型ロボットの予測モデルから[Ceres Solver](https://github.com/ceres-solver/ceres-solver)による最適化によって求めます。

## Input / Output

### Input

| **Name（Topic）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/map`          | `nav_msgs::msg::OccupancyGrid`                  | map_serverから受け取る2次元地図         | 

### Output

| **Name（Topic）**        | **Type**                                 | **Description**                                      | 
| -------------------- | ------------------------------------ | ------------------------------------------------ | 
| `/likelihood_map`      | `nav_msgs::msg::OccupancyGrid`         | 尤度場                                           | 

### [Parameters](../ike_nav_parameters/config/ike_controller_parameter.yaml)

| **Name（Parameter）**   | **Type**        | **Description**            | 
| ------------------- | ----------- | ---------------------- | 
| `map_frame`           | `std::string` | マップ座標系           | 

### Reference

* [Model Predictive Control: モデル予測制御入門](https://myenigma.hatenablog.com/entry/2016/07/25/214014)

## LICENSE

Apache License, Version 2.0に基づいています。
