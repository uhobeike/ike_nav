# ike_map_server

## Package overview
2次元マップサーバー（地図の管理）のROS 2 C++実装です。

2次元地図（PGM形式画像）からROS 2のメッセージで表現した2次元地図[`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)を出力します。

## Input / Output

### Input

### Output

| **Name（Topic）**        | **Type**                                 | **Description**                                      | 
| -------------------- | ------------------------------------ | ------------------------------------------------ | 
| `/map`          | `nav_msgs::msg::OccupancyGrid`                  | 2次元地図         | 

| **Name（Service）** | **Type**                                          | **Description**                             | 
| ------------- | --------------------------------------------- | --------------------------------------- | 
| `/get_map`          | `nav_msgs::msg::OccupancyGrid`                  |     2次元地図を渡す     | 

### [Parameters](../ike_nav_parameters/config/ike_map_server_parameter.yaml)

| **Name（Parameter）**   | **Type**        | **Description**            | 
| ------------------- | ----------- | ---------------------- | 
| `map_yaml_path`           | `std::string` | 2次元地図の情報が記載されたYAMLファイルパス           | 

## LICENSE

Apache License, Version 2.0に基づいています。
