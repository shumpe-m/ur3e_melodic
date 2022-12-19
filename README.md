# ur3e-melodic

- ubuntu18.04, melodic
- L字型のジグを装着したur3e

## Set up
- Dockerによる環境構築
```
git clone https://github.com/shumpe-m/ur3e_melodic.git
cd ur3e_melodic/Docker
./build.sh
./run.sh
```
## Usage
- 5つのターミナルでそれぞれコマンドを使用。(docker exec -it ${docker_container_id} bash)

(1) 1つめのターミナル:Gazebo起動
```
roslaunch ur_gazebo ur3e_bringup.launch
```

(2) 2つめのターミナル:Moveit!ノードを起動
```
roslaunch ur3e_moveit_config moveit_planning_execution.launch sim:=true
```

(3) 3つめのターミナル:Moveit!プラグインを含んだRvizを起動
```
roslaunch ur3e_moveit_config moveit_rviz.launch
```

(4) 4つめのターミナル:GazeboのUR3eのエンドエフェクタの軌道を描画(過去10sの軌道)
```
rosrun ur3e_motion plotting_rpy_2d.py
```

(5) 5つめのターミナル:UR3eの軌道生成コードの起動（y-z空間でエンドエフェクタが円を描く軌道）
```
rosrun ur3e_motion ur_planner.py
```


![demo](https://raw.github.com/wiki/shumpe-m/ur3e_melodic/images/ur3e_test.gif)

## Box sweep
- L字型のジグを装着したur3eの起動コード
それぞれ別のターミナルでコードを実行。
```
roslaunch ur3e_gazebo ur3e_sweep.launch
```

```
roslaunch ur3e_jig_moveit_config start_moveit.launch
```

```
rosrun ur3e_motion ur3e_sweep.py
```


![sweep](https://raw.github.com/wiki/shumpe-m/ur3e_melodic/images/sweep.gif)
