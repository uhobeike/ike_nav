<div align="center">
  <br>
  <img src="https://i.gyazo.com/368ca99e1363425f18462f1aaedf2df0.png" width="600">
  <h1>Simple ROS 2 Navigation Stack with C++ Implementation</h1>
  <strong><em>"IKE_NAV"</em>（ not <em>IKE IKE NAV</em> ）does not imply a superior navigation package</strong>
</div>
<br>
<p align="center">
  <a href="https://github.com/uhobeike/ike_nav/actions/workflows/build.yaml">
    <img src="https://github.com/uhobeike/ike_nav/actions/workflows/build.yaml/badge.svg" alt="Build">
  </a>
  <a href="https://github.com/uhobeike/ike_nav/actions/workflows/docker.yaml">
    <img src="https://github.com/uhobeike/ike_nav/actions/workflows/docker.yaml/badge.svg" alt="Docker">
  </a>
</p>

[![Image from Gyazo](https://i.gyazo.com/489e59d6dc9457911d26fcbcb926c120.png)](https://gyazo.com/489e59d6dc9457911d26fcbcb926c120)

## 📖 Package overview
*IKE_NAV*は、C++実装によるシンプルなROS 2ナビゲーションスタックです。  

2次元地図[`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)、2次元センサ情報[`sensor_msgs/LaserScan`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)、移動ロボットのオドメトリ情報[`tf2_msgs/TFMessage`](http://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html)を入力し、与えられた目標位置までに到達するための速度[`geometry_msgs/Twist`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html)を出力します。

[![](https://i.gyazo.com/54fed2846c77c3587dd778b544ef7709.png)](https://viewer.diagrams.net/?tags=%7B%7D&highlight=0000ff&edit=_blank&layers=1&nav=1&title=ike_nav_architecture.drawio#Uhttps%3A%2F%2Fraw.githubusercontent.com%2Fuhobeike%2Fike_nav%2Ffix%2Farchitecture-figure-2023-9-11-22-48%2F.github%2Fike_nav_architecture.drawio)

## ⚡ Quick demo *IKE_NAV*

```
docker run -it \
           --rm \
           --net=host \
           --env="DISPLAY=$DISPLAY" \
           --user $(id -u):$(id -g) \
           --mount type=bind,source=/usr/share/zoneinfo/Asia/Tokyo,target=/etc/localtime \
           --name ike_nav \
           ghcr.io/uhobeike/ike_nav:humble
```

https://github.com/uhobeike/ike_nav/assets/40545422/1f8b58f8-acae-4aea-9a2c-8f0c6799994f

## 🔨 Install and Build

```
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
git clone https://github.com/uhobeike/ike_nav.git
rosdep update
rosdep install -i -y --from-path src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 🧐 Known bug（これから修正予定）

* ウェイポイントをセット時、稀に出力され、ナビゲーションが停止する

```
[rviz2-5] [ERROR] [1693991158.604136358] [rviz2]: ItemIdentityException: Material with the name goal_flag_material_0.244069 already exists. in ResourceManager::add at ./.obj-x86_64-linux-gnu/ogre-v1.12.1-prefix/src/ogre-v1.12.1/OgreMain/src/OgreResourceManager.cpp (line 148)
```

* たまに自己位置推定が、落ちる（エラーの出力は無い）

## 👨‍💻 ToDo（現在、対応中）
* 実機で使用できるようにする
* 大きな地図でもサクサク動くようにする

## ⚖️ License
*IKE_NAV*は、[LGPL-3.0-or-laterとApache-2.0のライセンス](./LICENSE)で構成されています。  
詳しくは、各パッケージのpackage.xmlファイルを確認してください。
