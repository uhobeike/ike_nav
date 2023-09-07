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

2次元地図[`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)、
2次元センサ情報[`sensor_msgs/LaserScan`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)、
移動ロボットのオドメトリ情報[`tf2_msgs/TFMessage`](http://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html)を入力し、与えられた目標位置までに到達するための速度[`geometry_msgs/Twist`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html)を出力します。

[![](https://i.gyazo.com/33ed177f8a02236e186b19904da0cb1e.png)](https://viewer.diagrams.net/?tags=%7B%7D&highlight=0000ff&edit=_blank&layers=1&nav=1&title=ike_nav_architecture.drawio#R7V1bc6M6Ev41fkyKO%2FZjbs5M7c7Zqc3UzO6%2BuBSQbdVg5AOyE8%2BvXwkjDJKwZWLASVx5CAhoC%2FXXX7daFwb23eL1MQHL%2BTccwmhgGeHrwL4fWJZl%2BA79x0o22xLTdPxtySxB4bbM2BU8oT8wv5GXrlAI07xsW0QwjghaVgsDHMcwIJUykCT4pXrbFEdhpWAJZlAqeApAJJf%2BQiGZ56UjXmt24QtEszn%2FacvIrywAvzsvSOcgxC%2BlIvthYN8lGJPt0eL1Dkas%2BaoNM665WtQsgTHReWC9Wf47Hv89fQbx4%2F%2B%2B%2FrlZ%2FVr7V661FbMG0Sp%2F5by2ZMPbYE4WET0yB%2FbtGiYE0da5idAspmXPmBC8oBdgHN6w5mZlEQ5%2B06IEr%2BIQsh836FlKEvy7aEOXlsj1z18JhhWd5G%2FzCPECkmRDb3jZaWKYN24CI0DQuqo1kCt%2FVjxaSPuOEf1Vy8iRavpeLigHqjsaXbtVKSleJQHMHyy3syhrZEiyqpIISGaQSJLoQek1d0WZHo%2FQqekqdOpFtJFvpziracrtzPt7xeB3a%2B4O6dEs%2F589gsSC5wo6%2BGNM8tVW7g29gSr2dfuMIPbrPx4mf9385MLoCz6LP0DLpB%2BlZdu682IBo9S4luxwtYjGCVjQw9uXOSLwaQkCVv5C%2BYmWlaC8ZM0Pk4c1hV%2BqBKnNhOTH5pBdn3NL93Z33%2BEIJ1kl7KHB%2FuiVBV6D56xq7CyBtF3K55gAUjqnpAnL5zBE5VNmTpkZsVrnPFe6XGtGzFbh615Deq0a0aZ6WjKznX3My2RnOEa9pVVgfTSGjQsvZR60yiSe0N7anCTSmyioZUpyTIU6q5R0iFHsOkZZUMvWZAlqD6TKAiAHTAAZGSiQtEBhyB6XrDjjj6z13NuBe89krQjOmdWU6CGEU7DavjCKIrl0loAQ0UqU6OTBY3%2FsWgTSgqJ4FGFKUOYi7lFCKQJhVn1WBCL2qxFa%2FsxvZMdfchEnoA%2B3ii1bwR9DBXu4bXGHo%2FJ%2Fn5A7zIpaHJGr9bnjgKC2ucO5qFN2Bd7QaBygepKobhU6as8ZpAGIL96gT2%2FgCJGGIXuDUZfewFVFkidCW4Ipu9AbcMgrm3eRkmP7LxcstoFFUyA6S8aiZXcJRtMYHnZmM9p4S%2F23L5JfOQQG5fSSslUEV1L4g1KzOIaiWVrr7bn13YMQrdWBfumC1IIKV0%2BLxijixlWyMyEGyPMI0wi%2B5iIUQNyv22OCg6Ltm0YHuQ594QntWEDAgiin7VSVLSke%2FYYTqulJChMKdUm1O3VlJJkEec7YNA6nfCRKLRMepbW7u%2FH47q6Bxo9gJLFHr8i1dGt69cFYxcLGFAaTPRYoPcL0GIP1ZJHOUnrzIyTfdk8rHeTxFp3r3qvYt7HPvuErIv%2FJr7Dj%2F7Lya9%2FNT%2B9fS%2Ffdb%2FKTEKTzAnLs5DsgFEAMShQAFATHI2ZrnxqW0SmZ8K6GXQXpsCG5OMZ%2BOS2Ti6vytSpoH4R1umS9CkWg%2BAyC37MMYVfBlkdYvEgSEKccC4qwsWQW%2FwqCFRUebB7ZeNiuEttffLsx7HV2amPYbwtn7Qw5fkWS9U%2BDX1FO287RUHFzx8Gi7VSzP0XnrRwsqgYHRGM%2FmceyLWXMEOCUsLjBCs88ZhjVqKY%2BZvBHPQcJXL8fJ0howIsdBgmjHriT23fTjoQwiCjKaZ0rNbLENVx5pJ0eTaHi%2BKorW7NqfMT32rJmrd42C0IIRVIwicCGdcQuAdJpAySnByNvbNUdmbEn%2B3Y1NFE8ZS2A4ws6Pw46uesQiK%2BxC%2FK6xa6cy1JjFz9TYg0ieIHuh4Oue6LoSZTTNnTrZwxWoVvuZKlgW4%2BvcwrV3wOSxOivKZJEOW3H4ToT99rOWVh%2BtQ0cU85ZuKpxv6L2p09aaMXcrDtMrQFE6E8W3Gj2e9vMbzy4LMNxhNEUANDOb%2FQ%2FCOKrXPdbOo%2B1jaCPZT44qxiydlXQNdpqHM%2FXdA7C3Jv9mZ8UxilOuLP4J0hh8lR6%2FsxyPyfxJ34fQzp8UmbDwESa3NlxYOJpYu%2FdJB7PeXSyT4CahnUahEqC2oao7igjmWqjk0wtjswf428wTdkqtQsztgI870S4E%2BW0DDtfN4mLYkQQiJY4hdr440UBDiEv4y3JgfmdCvyFyPwOr0GCaEcPPhGwWMJyz7D8%2FAWtb0JrY3h2NgCjseqi6xiaj2GLpiqF1MoBGNduKaLm3dOLy3hLcsbtz2dYDSeUSBnqbieU8NX4hydEBdGkFYfxWVxEP%2Bg899E9vz7NXIXBOKL9swjNMQ6FXp0KLgdwqDHmcQHh6UdCBOfZeCTE7hSipqnKOOiFMadboGGIXVrFIirfUoQsokGfbpp4fSJmOVCNTy6onlA8sG%2B2a6e2p1cEL2mRkxWxJU9Xefr5pkg%2Fy2OWRRTDC%2B4AgTO8W201UOwhoQx5lmLZPBkctxlGxgiKt%2BU6uLFqVoppsoq4xitrryJHH8EpewrTu6ZRRj3TjGGy9Wo852%2Fl52OwQBED0BcYrSGTWqUh1bgAxSkBKGaKOIpoCqM5nPPvDcGmVvBzrqMvHWiGX7Ur3GPLzNPt2IypNfygvzask8nA3anLGZ2ZviwtfS0jEMconp2F0h7czm3s3JQmJ7FZayQ4ijpeg8cMaDTqVBtFjHk22pDXfaxRuio5po4V0rF52P6ZKYRXoKSQdJMSuOhYE517F%2B%2FsogHrDPpnxT4CvI%2FKDbbcLCZfOFVpF7%2B1hjFkjHbVQ%2FuaD6bzJduXLljzLlidzfe7peABW9QPfPjak5KleKrtYOzW%2BoF%2BfSZDmB%2BSxaj1E7T7CFFb1Axfr3kuTO%2FLQZB6sIKpabJku%2B2%2BZbSilCX%2BXpKlJLEPmi7moOllybaQ9m26WZ3j7ZfTcvp4WO%2BEZdBSbqEukzLFHIrDHG%2FCr3KU4wLkfoDMp4e9FciinLaBrDu9iE28rCdfeZrlllybg%2Bqc50%2BeFfBGDReQiMAT5bQNPHmMeLu7Q7%2FZqPYCL0fYXtXsfTXEUHfC%2F8X2z9T2Tc85jfFLgtq2ft3J1Ax65AWlRDtokuH4o%2Fz8Z53z3ytuXZH7hg2jJd85IKht3Gp3VhMYooA12WSOE%2FSnfpT50mt9D%2FgV6bLhMivfOSCoZfyO5H4rEpK6nUZdR%2B6pdXzUJRJGsWS4t6hrJPe4mApewCbbz3kypdEvfvl4mjANS0MVqqlC7alCXg7M1aBJkW1vNdwn6ZnG7tMDnK6cYeOPEcjSLK%2Fb72WN5JlhF3WXFGR7goIca9RU2ZIs2%2BhY2Vatp0vW6M9kGa1mKE47Ztkj91g4Acs6qgGebllWd9MZqhs0AwROCJ7MMIiadvn%2ByuX8wI8lKR%2Bu45dSCyJiDbPCUq3eA%2FMM67aePZ52xBBDlNQ26WhvocHUpIr5dLGqFoqXp5YZsAW%2FUb3UgzJSEk7SZJ2tV0zQbHZMhVoySA3b4jZrXA8dq2S3V8a1Ye23XHbyHSaIYunoiVu9%2Bn%2F669d8J6wiAGiYp6HCenb%2FlwTjZ04wWsOGkauULxAFtb68Xnd3yWARTtZQP0hSpRXFpbuHgXxJLp4%2B%2FHGrkGv8iR8x%2Bun6Gz9G8ym7R3Z%2Fjt5dzVZM3FVtBSFuJXu6WW66keFlK4jDALssti8PScmJjhqH8bn2dO0DK%2B97Y%2BsiZj8nAs%2BfunKFtRg2nyhXnmGu%2FD5Naysx9qxY7X6tPF4sVjEKxAXNz9ITKsdwWa3R0oL5wqKOWjDfNY41vlXa2Ye397djH9%2FjriYy3KbjXwfktM7tqhnrWo699UEvXd1yV2Bcm75R7eoovmx7sryYexoAWF4jAMiBKJ5OU9gSSOpng%2Bt%2BQ9t0a4ieMjYKSmy%2F9zPZ76E30oSTTjYRdGScBJSOu19O66xU33l5M%2BButl8gt8ZPMFmjALaAvfPOx%2FaIT0fc3ashPj2nZ3zK2div8XJFWB95RbID4x4QIIGIBfKD%2FZMDxDhqgcKQPS4t%2Bs1G3rIXdm8H7j2TtSI4j87NIq7iEw1iHDMp5bkHeVEHUWyR06yoTfUpPNUcBLutGNet3xRKl2dso4Zn1pCyBCIbXX7pERohnILV9q1L6NiVzhIQIlqJ8s5UHvtj1yKQprtwL8wIUmY9LuIeJXBLwPY9KwJZdyxCy5%2F5Y%2Bz4S34sA7FuD4f6iS%2B%2B8DEY13b0QCduoKwBOnqaYAaCHdnQfuL8Gw4hu%2BP%2F)

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

## ⚖️ License
*IKE_NAV*は、[LGPL-3.0-or-laterとApache-2.0のライセンス](./LICENSE)で構成されています。  
詳しくは、各パッケージのpackage.xmlファイルを確認してください。
