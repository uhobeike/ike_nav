# ike_controller

## Package overview
Model Predictive ControlのROS 2実装です。

ある経路[`nav_msgs/Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)に対して追従する速度[`geometry_msgs/Twist`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html)を、
差動二輪型ロボットの予測モデルから[Ceres Solver](https://github.com/ceres-solver/ceres-solver)による最適化によって求めます。

## Input / Output

### Input

| **Name（Service）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/get_twist`          | `ike_nav_msgs::srv::GetTwist`                  | ロボットの姿勢、目標経路を入力         | 

### Output

| **Name（Topic）**        | **Type**                                 | **Description**                                      | 
| -------------------- | ------------------------------------ | ------------------------------------------------ | 
| `/predictive_horizon`          | `nav_msgs::msg::Path`                  | 予測ホライズン         | 

| **Name（Service）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/get_twist`          | `ike_nav_msgs::srv::GetTwist`                  | ロボットの姿勢、目標経路から速度を出力         | 

### [Parameters](../ike_nav_parameters/config/ike_controller_parameter.yaml)

| **Name（Parameter）**   | **Type**        | **Description**            | 
| ------------------- | ----------- | ---------------------- | 
| `delta_time`           | `double` | 予測モデルでの経過時間           | 
| `predictive_horizon_num`           | `int` | 方策の数           | 
| `lower_bound_linear_velocity`           | `double` | 最低速度           | 
| `lower_bound_angular_velocity`           | `double` | 最低角速度           | 
| `upper_bound_linear_velocity`           | `double` | 最大速度           | 
| `upper_bound_angular_velocity`           | `double` | 最大角速度           | 
| `max_num_iterations`           | `double` | 最大反復回数           | 
| `recovery_rotate_velocity`           | `double` | 最適化計算が開始されるまで回転する時の角速度           | 
| `limit_absolute_rotate_velocity`           | `double` | 設定した以上の角速度は出力されないようにする           | 

### Reference

* [Model Predictive Control: モデル予測制御入門](https://myenigma.hatenablog.com/entry/2016/07/25/214014)
* [モデル予測制御（MPC）による軌道追従制御](https://qiita.com/taka_horibe/items/47f86e02e2db83b0c570)

## LICENSE

Apache License, Version 2.0に基づいています。
