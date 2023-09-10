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

[![](https://i.gyazo.com/4bb55f279cbd0456c2a5e8832ecdac8c.png)](https://viewer.diagrams.net/?tags=%7B%7D&highlight=0000ff&edit=_blank&layers=1&nav=1&title=ike_nav_architecture.drawio#R7V1bc9o8Gv41XCbjs%2BEyTULa2e23nU2n3d0bRrEFaGosPluQpL9%2BJWMZW5JBOD6QlMlFbGG%2FGL3P%2B7wHHTyyb1cvDwlYL7%2FiEEYjywhfRvbdyLJM2zPoP9byumvxfXPXsEhQuGsy9g2P6DfM7%2BStGxTCNG%2FbNRGMI4LW1cYAxzEMSKUNJAl%2Brl42x1FYaViDBZQaHgMQya0%2FUUiWeeuEPzX74DNEiyX%2FasvIP1kBfnXekC5BiJ9LTfb9yL5NMCa7o9XLLYxY71U7ZlrzafFkCYyJzg3b1%2FW%2F4%2Bnf8ycQP%2Fzvy%2B%2Bbzc%2Btf2W6OzFbEG3ynzyyvIgK%2FDTHVC59bK4T7%2B8Ne9RP5v6QHi3y%2F9ktSGx4YgLIa1SVwCRf7eTe0Avoj3jZ3SOI%2FfKP%2B9lfNz%2B4MPrTnsQvoG3Sl9K23bPzZqvyFBZVxJodblbRNAErevjpeYkIfFyDgLU%2FUyjTtiVZRfTMpIdrjGICk%2Fst7WoGKIO2pSTBvwpQ2ExIfmyO2edLjgpvf%2FUtjnCSPYQ9Ntgf%2FWSFt%2BApezR2lkDaL%2BVzTAApnVP7guVzGKLyaYSDXzDMnzq3idLHMmRyFG1hQuBLqSmH0APEK0iSV3pJ%2Fuk4R%2FNr9fS5ZBw%2BN%2Fpl2TAMJ28FuUkuCtl70NKDHLenYNhQYFjQeUmZ7LciauE3EVrEtO0JE4JXrC%2Fj8IZRBmtjHZl1%2FyYOsw6VVe4e6lAYVnhF7s5Sh%2FE%2BTGAECNpWmUfVW7m0bwyVe81YVc24ntDfKd4kAcxvKvOEIMf0jggiIFlAIgnKdFf8wubqdEyFOquUdIxR7DpGWVHL1mQJag%2BkygIgB0wAGRkokLRCYchul6w444%2Bs99xPI%2FeOydoQnDOrKdFDCOdgs%2FvBKIrk1kUCQkQfokQn9x77Y59FIC0oinscU4IyF3GHEkoRCLPHZ00gYt8aofWP%2FEJ2%2FDkX0QJ9uFVs2Qr%2BGCvYw%2B2KOxyV%2F%2FsDucOsqMURuVqfO44I6po7nIs6ZVfgjY1rtx1n4I17VuikO2eQBiC%2BeIMhvYEjRBqG7A0mfXoDVxVJtoS2BFN2oRfgkD9sniIlp%2BYvFyx2gUVTIDpLxqJl9wlG0xgfd2YL2nlr%2FV9fFEpyCIzKpQhlrwiupPAHpW5xDEW3dJbtufXpQYi26kC%2F9IHUgwpXT5umKOLGVbIzIQbI6wjzCL7kIhRAPKzbU4KDou%2BbRge8BifcoR0LCFgQ5XQcCpi2pHj0C86opmcpTCjUJdXu1ZWRZBLk9UXTOF7ykSi1THiU1m5vp9Pb2wYaP4GRxIxeUWvp1%2FTqg7GKhU0pDGYHLFC6hekxBtvZKl2k9OIHSL7u71Y6yNMtOte9V7Fv45B9wxdE%2FpN%2Fwo7%2Fy9qvfTc%2FvXspXXf3mp%2BEIF0WkGMn3wChAGJQogCgIDgdMTv71LCMXsmEpxp2FaTjhuRSOJEaOR2Ti6vytSpoH4V1umZZhSJQfALBr0WGsKtgxyMsXiQJiFOOBUXYWDKLfwXBhgoPXh%2FY2Mn%2BIXbf%2BHZjOOjs1MZw2BbO2hly%2FIok67eDX1FO187RUHFzz8Gi7VSrP0XyVg4WVYMDorG35rFsSxkzBDglLG6wwjOPGSY1qqmPGfzJwEEC1%2B%2FHCRIa8GKPQcJkAO7k9t00kRAGEUU5nXOlRpW4hitPtNOTKVQcX3Vla1aNj%2FheV9aslW2zIIRQJAWzCLyyROwSILUbIDkDGHljq%2B7JjD3Zt6uhieI56wEcX9D5cdDJXYdAfI1dkNcvduValhq7%2BIkSaxDBC3Q%2FHHTdlqInUU7X0K2fMViFbjnJUsG2Hl%2FnFKq%2FBySJ0V9TJIlyuo7DdSbudV2zsPxqHzimXLNwVeN%2BxdO3X7TQirlZOkytAUTodxbcaOa9XdY37l1W4TjBaAoAaNc3hh8E8VWu%2By3JY20n6GOZD84qhqxdFXSNrjrH8zWdgzD35nDlJ4VxihPuLP4JUpg8lu4%2Fs9pPK%2F7EH2JIh0%2FKbBiYSJM7ew5MPE3svZvC4zmPTg4JUNOw2kGoJKhriOqOMpK5NjrJ3OLI%2FD79CtOUrWi6MGMnwPNawp0op2PY%2BbpFXBQjgkC0xinUxh9vCnAIeRvvSQ7Mb1TgT0SWt3gLEkQTPfhIwGoNy5lh%2Bf4LWt%2BE1sbw7G0ARmPVRd8xNB%2FDFk1VCqmVAzCu3VFEzdPTi8t4S3HGHc5nWA0nlEgV6n4nlPiqMVLlhKggmnXiMP4UFzEMOs99dM%2BvLzNXYTCNaH4WoSXGoZDVqeByBIcaYx4XELY%2FEiI4z8YjIXavEDVNVcVBL4xpb4GGIaa0ikVUvqUIWUSDbm%2BaeH0hZj1SjU%2BuqJ5QPLJvdmundqdXBK9pk5M1sSVPV3n5%2BaYoP8tjlkUUwxtuAYELvF9tNVLsIaEMedZi2zIZnbYZRsYIil%2FLdXBj1awU02QVcY1X1l9FjT6Cc3YXplfNo4x65hnDZOvVeM3fys%2BnYIUiBqDPMNpCJrVKQ6pxAYpTAlDMFHES0RRGc7zmPxiCTa3g51xHX3rQTLG%2FT4V7bJl5%2Bh2bMbWGH%2FTXhvUyGbg%2FdTmTM9OXpaWvdQTiGMWLs1Davdu7jZ2b0uQiNuuNBEdRz2vwmAFNJr1qo4gxz0Yb8rqPLUo3JcfUs0J6Ng%2FbPzOF8AcoKSR9TQlc9ayJ3r2Ld3bRgHUG%2BVmxjwDPUbnBlrvF5AunKv3id9YxhozRvjK0L%2FlgOl%2ByfUnBmqdgdTY%2F7JaCR2xRP%2FDha09KluKptoOxO8sD%2FfpKhjA%2FJItR6ydoDxGidqgZvl7zXJjel4Mg9WAFU9NszXZmfctoRalK%2FK0kS0liH7RczEEzyJJtoezbdLM6xzssp%2BPy8bjeCcugpdxCXSZliiUUhznehF%2FlKMcFyMMAmU8PeyuQRTldA1l3ehGbeFlPvvI0yx25NgfVOc%2BfPCvgTRouIBGBJ8rpGnjyGPFud4dhq1HdBV6OsL2qOfhqiLHuhP%2BL7Z%2Bp7Zue047xS4K6tn7dydQMeuQZpUQ7aJLh%2BL18%2F586539Q3Loi940bRku%2Bc0RQ17jVTlYTGKKAddlsiRP0u36U%2BZK1vgf8inTZcJmV7xwR1DF%2BJ3LeioSibq9R14l7ap0edYmEUSwZHizqmsgZF1PBM3jN9nOezWn0i58%2FniZMw9JQhWqqUHeqkJcDczVoUmTXWw0PSXqmsX%2F1AKcrZ9z4ZQSyNMub9Et%2B8sywi7pLCrI9QUGONWmqbEmWbfSsbKvW0yVb9Hu2jjYLFKc9s%2ByJeyy0wLKOaoCnX5bV3XSG6gYtAIEzgmcLDKKmKd9fuZzv%2BKEk5cMlfim1ICI%2BYdZYeqr3wDzjuq1nT6cdMcQQJXVNOtpbaDA1qWI%2BXayqheJ12zIDtuA3qpd6VEZKwlmabLP1iglaLE55oI4MUsO2uM0a12PHKtntlXFtWIctl518gwmiWDp54tag%2Fp9%2B%2BzXfCasIABrWaaiwgd3%2FpcD4JxcYrXHDyFWqF4iCOl9er7u7ZLAKZ1uoHySpyori0t3jQL4UF9sPf9wq5Bq%2F4keMfvp%2Bx4%2FRfMruienPybur2YqJu6qtIMStZNub5aYbGV62gjgOsMti%2B%2FKQlFzoqHEYf9aerkNg5X1vbF3E7OdE4PldV66wFsPmE%2BXKM8yV76fpbCXGgRWr%2Fa%2BVx6vVJkaBuKD5SbpD5RguqzU6WjBfWNRJC%2Bb7xrHGu0p7e%2FH24X4c4n3c1UKG23T864iczrldNWNdy7F3Puilq1vuCoxr0zeqqY7izbat1cXcdgBgeY0AIAeieD5PYUcgqZ8NrvsObdOtIfrveI2CEtsffE32e8hGmnBSaxNBJ0YroHTcw3I6Z6X65OXNgLvZvYHcmj7CZIsC2AH2zrseOyA%2BHXF3r4b49JyB8SlXY7%2FE6w1hOfKGZAfGHSBAAhEL5EeHJweIcdQKhSG7XVr0m428ZT%2FY%2FTRy75isDcF5dG4WcRWfaBDjmEkpzz3Im3qIYouaZkVtqlfhqeYg2F3FuG79plC6PGMbNTyzhZQlEHnV5ZcBoRHCOdjsfnUJHfvWRQJCRB%2BivDOVx%2F7YZxFI0324F2YEKbMeF3GHErgjYPuONYEsHYvQ%2Bkd%2BGzv%2BnB%2FLQKzbw6F%2B4osvvAzGtR090IkbKLcHOpVzGyqvMk93Bi0lUKYvzJ52J5PGswgn4quaJ22Nz9HTBDPD3l9Oc%2F%2FlVxxCdsX%2FAQ%3D%3D)

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
