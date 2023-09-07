# ike_planner

## Package overview
A*[^1]およびDijkstra[^2]探索のROS 2実装です。

地図上での自己位置から、与えられた目標位置までの最短経路を求めます。

## Input / Output

### Input

| **Name（Topic）**        | **Type**                                 | **Description**                                      | 
| -------------------- | ------------------------------------ | ------------------------------------------------ | 
| `/costmap_2d`          | `nav_msgs::msg::OccupancyGrid`                  | すべてのレイヤーを統合したコストマップ         | 

| **Name（Service）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/get_path`          | `ike_nav_msgs::srv::GetPath`                  | ロボットの姿勢と目標位置を入力         | 

### Output

| **Name（Topic）**        | **Type**                                 | **Description**                                      | 
| -------------------- | ------------------------------------ | ------------------------------------------------ | 
| `/plan_path`          | `nav_msgs::msg::Path`                  | 求めた最短経路         | 
| `/planner_searched_map`          | `nav_msgs::msg::OccupancyGrid`                  | 最短経路を求めるのに探索したエリア         | 

| **Name（Service）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/get_path`          | `ike_nav_msgs::srv::GetPath`                  | ロボットの姿勢と目標位置間の最短経路を出力         | 

### [Parameters](../ike_nav_parameters/config/ike_planner_parameter.yaml)

| **Name（Parameter）**   | **Type**        | **Description**            | 
| ------------------- | ----------- | ---------------------- | 
| `use_dijkstra`           | `bool` | Dijkstra法で最短経路を求めるか           | 
| `publish_searched_map`           | `int` | 探索エリアをパブリッシュするか           | 
| `update_path_weight`           | `double` | 経路のスムージングのために、データをどれだけ元からずらすかの重み           | 
| `smooth_path_weight`           | `double` | 経路をどれだけ、スムージングするかの重み           | 
| `iteration_delta_threshold`           | `double` | 反復の閾値           | 

## Reference

* [MATLABよる経路平滑化(Path Smoothing)プログラム](https://myenigma.hatenablog.com/entry/20140510/1399694663)
* [AtsushiSakai/PythonRobotics PathPlanning/AStar/a_star.py](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py)

## LICENSE

Apache License, Version 2.0に基づいています。

### Citation
[^1]: P. E. Hart, N. J. Nilsson and B. Raphael: ‘‘A Formal Basis for the Heuristic Determination of Minimum Cost Paths,’’ in IEEE Transactions on Systems Science and Cybernetics, vol. 4, no. 2, pp. 100-107, July 1968.
[^2]: E. W. Dijkstra: ‘‘A note on two problems in connexion with graphs,’’ Numer. Math., vol. 1, no. 1, pp. 269–271, Dec 1959.