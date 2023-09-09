# ike_nav_server

## Package overview
navigation server（ロボットの目標位置到達までの管理）のROS 2 C++実装です。

目標位置[`geometry_msgs/PoseStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)が与えられると、経路計画と経路追従を開始し、速度[`geometry_msgs/Twist`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html)を出力します。

## Input / Output

### Input

| **Name（Topic）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/goal_pose`          | `geometry_msgs::msg::PoseStamped`                  | 目標位置         |

| **Name（Service）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/get_path`          | `ike_nav_msgs::srv::GetPath`                  | 経路         | 
| `/get_twist`          | `ike_nav_msgs::srv::GetTwist`                  | 速度         | 

| **Name（Action）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/navigate_to_goal`          | `ike_nav_msgs::action::NavigateToGoal`                  | 目標位置         | 

### Output

| **Name（Topic）**        | **Type**                                 | **Description**                                      | 
| -------------------- | ------------------------------------ | ------------------------------------------------ | 
| `/cmd_vel`          | `geometry_msgs::msg::Twist`                  | 速度         | 

| **Name（Action）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/navigate_to_goal`          | `ike_nav_msgs::action::NavigateToGoal`                  | ロボットの姿勢、目標経路から速度およびフィードバックを出力         | 

### [Parameters](../ike_nav_parameters/config/ike_nav_server_parameter.yaml)

| **Name（Parameter）**   | **Type**        | **Description**            | 
| ------------------- | ----------- | ---------------------- | 
| `ike_nav_server_loop_hz`           | `double` |      経路計画と経路追従の実行周期      | 
| `goal_tolerance_xy`           | `double` | ゴール判定の許容値           | 
| `publish_stop_velocity_hz`           | `double` | 停止をするための速度のパブリッシュ周期           | 

## LICENSE

Apache License, Version 2.0に基づいています。
