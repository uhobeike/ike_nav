# ike_waypoint_follower

## Package overview
waypoint follower（複数の目標位置の管理）のROS 2 C++実装です。

複数の目標位置[`ike_nav_msgs/Waypoints`](https://github.com/uhobeike/ike_nav/blob/main/ike_nav_msgs/msg/Waypoints.msg)を通過するようにします。

## Input / Output

### Input

| **Name（Service）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/load_waypoint_yaml`          | `ike_nav_msgs::srv::LoadWaypointYaml`                  | ウェイポイントの読み込み         |
| `/start_waypoint_follower`          | `std_srvs::srv::Trigger`                  |   waypoint followerの開始       | 
| `/stop_waypoint_follower`          | `std_srvs::srv::Trigger`                  | waypoint followerの停止（再開すると停止前の通過点から開始）       |
| `/cancel_waypoint_follower`          | `std_srvs::srv::Trigger`                  | waypoint followerの終了（再開すると最初の通過点から開始）         |

### Output

| **Name（Topic）**        | **Type**                                 | **Description**                                      | 
| -------------------- | ------------------------------------ | ------------------------------------------------ | 
| `/waypoints`          | `ike_nav_msgs::msg::Waypoints`                  | 複数の目標位置         | 

| **Name（Action）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/navigate_to_goal`          | `ike_nav_msgs::action::NavigateToGoal`                  | 目標位置         | 

### [Parameters](../ike_nav_parameters/config/ike_waypoint_follower_parameter.yaml)

| **Name（Parameter）**   | **Type**        | **Description**            | 
| ------------------- | ----------- | ---------------------- | 
| `waypoint_yaml_path`           | `string` |      経路計画と経路追従の実行周期      | 
| `waypoint_radius`           | `double` | ウェイポイント到達判定の許容値           | 

## LICENSE

Apache License, Version 2.0に基づいています。
