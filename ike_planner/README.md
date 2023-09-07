# ike_planner

## Package overview
A*[^1]およびDijkstra[^2]探索のROS 2実装です。

地図上での自己位置から、与えられた目標位置までの最短経路を求めます。

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

### [Parameters](../ike_nav_parameters/config/ike_planner_parameter.yaml)

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

## Reference

* [MATLABよる経路平滑化(Path Smoothing)プログラム](https://myenigma.hatenablog.com/entry/20140510/1399694663)
* [AtsushiSakai/PythonRobotics PathPlanning/AStar/a_star.py](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py)

## LICENSE

Apache License, Version 2.0に基づいています。

### Citation
[^1]: P. E. Hart, N. J. Nilsson and B. Raphael: ‘‘A Formal Basis for the Heuristic Determination of Minimum Cost Paths,’’ in IEEE Transactions on Systems Science and Cybernetics, vol. 4, no. 2, pp. 100-107, July 1968.
[^2]: E. W. Dijkstra: ‘‘A note on two problems in connexion with graphs,’’ Numer. Math., vol. 1, no. 1, pp. 269–271, Dec 1959.