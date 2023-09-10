# ike_nav_rviz_plugins

## Package overview
RVizによるウェイポイント/ナビゲーションの管理および可視化のROS 2 C++実装です。

## How to use
[![Image from Gyazo](https://i.gyazo.com/7ca70b30986a4ed68739033c7d1c6f4b.png)](https://gyazo.com/7ca70b30986a4ed68739033c7d1c6f4b)

### Start Waypoint Folloer（ウェイポイントによるナビゲーションの開始）
①のボタンを押すと開始します。

### Stop Waypoint Folloer（ウェイポイントによるナビゲーションの停止）
②のボタンを押すと停止します。  
①のボタンを押して再開すると、停止前のウェイポイントを目指します。  

### Stop Waypoint Folloer（ウェイポイントによるナビゲーションの中止）
③のボタンを押すと中止します。  
①のボタンを押して再開すると、一番最初のウェイポイントを目指します。  

### New Waypoint Set（新しいウェイポイントのセット）
⑦にあるツールで新しいウェイポイントの配置ができます。  

### Delete Waypoint （現在の一番最後のウェイポイントを削除）
⑤を押すことでウェイポイントを削除します。

### Delete All Waypoints（すべてのウェイポイントの削除）
⑤を押すことで全てのウェイポイントを削除します。  
ナビゲーション中に行うと、ナビゲーションが中止されます。

### New Waypoint Set（YAMLファイルにウェイポイントの情報を保存）
④を押すことで、現在のウェイポイントを保存するためのウィンドウが表示されます。  
ファイル名を指定して保存することができます。
[![Image from Gyazo](https://i.gyazo.com/d45f236c609b8a22874b4d65d6fe1290.png)](https://gyazo.com/d45f236c609b8a22874b4d65d6fe1290)

### New Waypoint Set（YAMLファイルからウェイポイントの情報を読み込む）
⑧を押すことで、YAMLファイルに記述されたウェイポイントを読み込むためのウィンドウが表示されます。  
ファイル名を指定して読み込むことができます。
[![Image from Gyazo](https://i.gyazo.com/569ba0b80d2afe22e56ea404e8ca5ead.png)](https://gyazo.com/569ba0b80d2afe22e56ea404e8ca5ead)

## LICENSE

Apache License, Version 2.0に基づいています。

