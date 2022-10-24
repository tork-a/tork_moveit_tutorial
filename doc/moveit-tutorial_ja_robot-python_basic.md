
# プログラムでロボットを動かす

物理シミュレータと MoveIt! を使って
プログラムからロボットを操作します．


## プログラムを入力して実行する

1行もしくは数行ごとにプログラムを入力して実行し，各コマンドで何をしているのかを見てみます．

シミュレータや MoveIt! を起動し，
それらを起動したターミナルとは別に新たにターミナルを起動して
ROS の環境設定と対話的にプログラミングを行えるように準備します．

<$if <$ROS_DISTRO>==indigo||<$ROS_DISTRO>==kinetic||<$ROS_DISTRO>==melodic>

### NEXTAGE OPEN の場合

「シミュレータと MoveIt! の起動」の
「[NEXTAGE OPEN - hrpsys シミュレータ](moveit-tutorial_ja_robot-simulator.md#start-nextage-hrpsys-simulator)」もしくは
「[NEXTAGE OPEN - Gazebo シミュレータ](moveit-tutorial_ja_robot-simulator.md#start-nextage-gazebo-simulator)」を参照して
hrpsys か Gazebo のどちらか一方のシミュレータと MoveIt! を起動します．

次に1行もしくは複数行ごとにプログラミングとその実行を行う
対話的プログラミングコンソールを起動します．

**ターミナル** : 対話的プログラミングのコンソールの起動
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ rosrun tork_moveit_tutorial demo.py
```

#### 特定の関節を動かす

プログラムから関節を動かすなどをするために「右腕」の `group` を作成します．
次の `In[1]:` 以下につづくプログラムを入力して [Enter/Return] キーを押して
1行ずつ実行してください．

- 補足 : `In [1]:` の `[ ]` の中の数字はプログラム入力するたびに更新されます．本チュートリアルにある入力例にある数字と一致しない場合がありますがそれは問題ありませんので，そのままチュートリアルを進めてください．

```python
In [1]: group = MoveGroupCommander("right_arm")
[ INFO] [1511506815.441893962, 135.795000000]: TrajectoryExecution will use new action capability.
[ INFO] [1511506815.442105792, 135.795000000]: Ready to take MoveGroup commands for group right_arm.

In [2]:
```

- 注意：以下のように`TrajectoryExecution will use old action capability.`と表示される場合も有りますが問題ありません．
```python
In [1]: group = MoveGroupCommander("right_arm")
[ INFO] [1511506815.441893962, 135.795000000]: TrajectoryExecution will use old action capability.
[ INFO] [1511506815.442105792, 135.795000000]: Ready to take MoveGroup commands for group right_arm.
```

- 注意 : MINAS TRA1 の場合はグループ名が異なります．
```python
In [1]: group = MoveGroupCommander("manipulator")
```

<$ifeq <$ROS_DISTRO>|melodic>

- 注意 : myCobot の場合はグループ名が異なります．
```python
In [1]: group = MoveGroupCommander("arm_group")
```
<$endif>

グループ `group` に含まれる関節の名前を `get_joints()` で調べます．

```python
In [2]: group.get_joints()
Out[2]:
['RARM_JOINT0',
 'RARM_JOINT1',
 'RARM_JOINT2',
 'RARM_JOINT3',
 'RARM_JOINT4',
 'RARM_JOINT5']

In [3]:
```

上は NEXTAGE OPEN の右腕の場合の出力結果で
`RARM_JOINT0`〜`RARM_JOINT5` の6つの関節があることがわかります．

肘関節に相当する `RARM_JOINT2` を動かしてみます．

`set_joint_value_target()` を使って関節の目標値を設定します．
`set_joint_value_target()` に関節名 `'RARM_JOINT2'` と
関節角度を `-2.0`（単位ラジアン[rad]）を渡します．

```python
In [3]: group.set_joint_value_target( 'RARM_JOINT2', -2.0 )
```

関節角度目標を設定したので `go()` で動かします．

```python
In [4]: group.go()
Out[4]: True
```

正常に動作が完了すると `True` が返ってきます．
肘関節 `RARM_JOINT2` が少し屈曲したと思います．

同じ要領で他の右腕の関節もそれぞれ動かしてみます．

```python
In [5]: group.set_joint_value_target( 'RARM_JOINT3', -0.78 )
In [6]: group.go()
Out[6]: True

In [7]: group.set_joint_value_target( 'RARM_JOINT4', 0.78 )
In [8]: group.go()
Out[8]: True

In [9]: group.set_joint_value_target( 'RARM_JOINT5', 0.78 )
In [10]: group.go()
Out[10]: True
```


#### 腕全体の関節を動かす

`set_joint_value_target()` は1つの関節だけでなく
腕全体の関節角度目標値のリストを渡すことで複数の関節を同時に動かすこともできます．
`group.get_joints()` で調べたように NEXTAGE OPEN の右腕には6つの関節があるので
6つ分の関節角度目標値 [rad] を持ったリストを `set_joint_value_target()` に渡します．

```python
In [11]: group.set_joint_value_target( [ 0.0, 0.0, -1.57, 0.0, 0.0, 0.0 ] )
In [12]: group.go()
Out[12]: True
```

NEXTAGE OPEN ロボットは初期姿勢に戻っていることと思います．


#### 手先の位置を指定して動かす

手先（エンドエフェクタリンク）の位置を指定して腕を動かしてみます．

`set_position_target()` を使います．
手先の目標位置の座標 ( x, y, z ) をリスト `[ X, Y, Z ]` で
`set_position_target()` に渡します．

位置座標の単位はメートル [m] です．
座標原点は NEXTAGE OPEN の場合，腰部リンク (`/WAIST`) にあります．
各軸の方向は下記の右手座標系です．

- X方向 : 正 = 前 / 負 = 後
- Y方向 : 正 = 左 / 負 = 右
- Z方向 : 正 = 上 / 負 = 下


```python
In [21]: group.set_position_target( [ 0.3, -0.3, 0.3 ] )
```

`go()` を使って実行します．

```python
In [22]: group.go()
Out[22]: True
```

手先の「位置」しか指定していないので
手先の「姿勢」は思わぬ方を向いていることもあります．


#### 手先の姿勢を指定して動かす

手先の姿勢を指定してロボットを動かしてみます．

`set_rpy_target()` を使います．
手先の目標姿勢の角度 ( roll, pitch, yaw ) をリスト `[ R, P, Y ]` で
`set_rpy_target()` に渡します．

```python
In [31]: group.set_rpy_target( [ 0.0, -2.36, 0.0 ] )
```

`go()` を使って実行します．

```python
In [32]: group.go()
Out[32]: True
```

手先の「姿勢」しか指定していないので
手先の「位置」は思わぬところにあることもあります．

- 注意：以下のようなエラーメッセージが表示された場合は，再度 `group.go()` をお送りください．
```
[ INFO] [1515668193.145149146, 166.319999999]: ABORTED: Solution found but controller failed during execution
```

#### 手先の位置と姿勢を指定して動かす

手先の位置と姿勢を同時に指定して腕を動かすことができます．

`set_pose_target()` を使います．
次のいずれか1つを `set_pose_target()` に渡して手先の位置と姿勢を指定します．

- 位置座標と Roll/Pitch/Yaw 姿勢角の6つの数値のリスト `[x, y, z, rot_x, rot_y, rot_z]`
- 位置座標とクォータニオンの7つの数値のリスト `[x, y, z, qx, qy, qz, qw]`
- `Pose`型
- `PoseStamped`型

どれを渡しても `set_pose_target()` 内で判断して適切な処理がなされます．

- 参考 : GitHub - move_group.py 内の `set_pose_target()` の定義

[https://github.com/ros-planning/moveit/blob/1.0.2/moveit_commander/src/moveit_commander/move_group.py#L252][6cf69e4b]

  [6cf69e4b]: https://github.com/ros-planning/moveit/blob/1.0.2/moveit_commander/src/moveit_commander/move_group.py#L252 "set_pose_target()"

それでは右手先の位置と姿勢を指定して腕を動かしてみます．
まずは 位置とRPY角 を `set_pose_target()` に渡して動作させます．
```python
In [41]: group.set_pose_target( [ 0.4, -0.4, 0.15, 0.0, -1.57, 0.0 ] )
In [42]: group.go()
Out[42]: True
```

もう1つ位置・姿勢を指定して腕を動かしてみます．

```python
In [43]: group.set_pose_target( [ 0.3, -0.3, 0.5, 0.0, -3.14, 0.0 ] )
In [44]: group.go()
Out[44]: True
```

今度は 位置とクォータニオン を `set_pose_target()` に渡して腕を動かしてみます．

```python
In [45]: group.set_pose_target( [ 0.4, -0.4, 0.15, 0.0, -0.707, 0.0, 0.707 ] )
In [46]: group.go()
Out[46]: True
```

```python
In [47]: group.set_pose_target( [ 0.3, -0.3, 0.5, 0.0, -1.0, 0.0, 0.0] )
In [48]: group.go()
Out[48]: True
```

> - クォータニオン（四元数/しげんすう）
>     - 空間上の姿勢を表現するための4つの成分をもつベクトル
>         - 複素数から拡張された数体系
>     - 回転の結合・補間の計算が容易
>     - Roll/Pitch/Yaw のオイラー角により姿勢を表現した場合と比べて
>         - 長所
>             - ジンバルロックという特異点がない
>             - 計算が速い
>         - 短所
>             - 直感的に分かりにくい
>     - 比較的理解しやすい Roll/Pitch/Yaw からクォータニオンに変換することも可能
>         - 3D Rotation Converter
>             - https://www.andre-gaschler.com/rotationconverter/
>         - ROS Wiki - Quaternion Basics
>             - http://wiki.ros.org/tf2/Tutorials/Quaternions
>     - ロボット以外では航空宇宙や 3D グラフィックスなどの分野で用いられる

更に `Pose` 型を `set_pose_target()` に渡して腕を動かしてみます．

```python
In [49]: pose_target_1 = Pose()

In [50]: print( pose_target_1 )
position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0

In [51]: pose_target_1.position.x = 0.4
In [52]: pose_target_1.position.y = -0.4
In [53]: pose_target_1.position.z = 0.15
In [54]: pose_target_1.orientation.x = 0.0
In [55]: pose_target_1.orientation.y = -0.707
In [56]: pose_target_1.orientation.z = 0.0
In [57]: pose_target_1.orientation.w = 0.707

In [58]: print( pose_target_1 )
position:
  x: 0.4
  y: -0.4
  z: 0.15
orientation:
  x: 0.0
  y: -0.707
  z: 0.0
  w: 0.707

In [59]: group.set_pose_target( pose_target_1 )
In [60]: group.go()
Out[60]: True
```

ポーズをもう1つ指定して腕を動かします．

```python
In [61]: pose_target_2 = Pose()

In [62]: print( pose_target_2 )
position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0

In [63]: pose_target_2.position.x = 0.3
In [64]: pose_target_2.position.y = -0.3
In [65]: pose_target_2.position.z = 0.5
In [66]: pose_target_2.orientation.y = -1.0

In [68]: print( pose_target_2 )
position:
  x: 0.3
  y: -0.3
  z: 0.5
orientation:
  x: 0.0
  y: -1.0
  z: 0.0
  w: 0.0

In [69]: group.set_pose_target( pose_target_2 )
In [70]: group.go()
Out[70]: True
```

`set_pose_target()` に渡せるもう1つの型 `PoseStamped` 型については

- 発展的なロボットプログラミング - より複雑なロボット動作計画 / 姿勢の参照座標を指定する

にて説明します．


#### 直線補間軌道でロボットを動かす

`group.plan()` や `group.go()` を用いた動作計画では動作開始姿勢と目標姿勢の間の動作は
各関節の開始角度と目標角度の間を補間した動作として計画されます．
このことは開始姿勢や目標姿勢として指定した姿勢以外の動作途中におけるエンドエフェクタの姿勢は
保証されないことを意味します．

エンドエフェクタを目標姿勢間で直線的に動作させたい場合は
`group.compute_cartesian_path()` を用いて動作計画をします．
`compute_cartesian_path()` はその名前のとおり，
直行座標（=デカルト座標: Cartesian Coordinates）における補間軌道（Path）を作成します．

`group.plan()` や `group.go()` で作成された動作計画（画像:左）と
`group.compute_cartesian_path()` で作成された動作計画（画像:右）を比較すると
次のようになります．

![MoveIt! - Trajectory Comparison / compute_cartesian_path()](images/nextage_moveit_cartesian-path_linear-trajectory_comparison.png)

`compute_cartesian_path()` の具体的な使用方法は本項に続く項目の

- 連続した指令をロボットに送る
- 四角形や円に沿ってエンドエフェクタを動かす

を通して学習します．


#### 連続した指令をロボットに送る

ロボットの複数の異なる姿勢を指示して動作計画と実行を行います．

複数の姿勢を指定した動作計画を行う場合も直線補間軌道でロボットを動かす場合と同じ 
`compute_cartesian_path()` を用います．

`compute_cartesian_path( self, waypoints, eef_step, jump_threshold, avoid_collisions = True )` 
には次のものを渡します．

- `waypoints` : エンドエフェクタが経由する姿勢のリスト
- `eef_step` : エンドエフェクタの姿勢を計算する間隔の距離
- `jump_threshold` : 軌道内の連続する点間の最大距離（`0.0` で無効）
- `avoid_collisions` : 干渉と運動学上の制約チェック（デフォルトは `True` でチェックする）

本チュートリアル手順に則って進めている場合，
複数の姿勢のリスト `waypoints` が用意されているので内容を確かめてみます．

```python
In [81]: print( waypoints )
[position:
  x: 0.4
  y: -0.4
  z: 0.15
orientation:
  x: 0.0
  y: -0.707
  z: 0.0
  w: 0.707, position:
  x: 0.4
  y: -0.2
  z: 0.15
orientation:
  x: 0.0
  y: -0.707
  z: 0.0
  w: 0.707, position:
  x: 0.3
  y: -0.3
  z: 0.5
orientation:
  x: 0.0
  y: -1.0
  z: 0.0
  w: 0.0, position:
  x: 0.3
  y: -0.5
  z: 0.5
orientation:
  x: 0.0
  y: -1.0
  z: 0.0
  w: 0.0]

In [82]:

```

姿勢のリスト `waypoints` を `compute_cartesian_path()` に渡して動作計画を作成します．

```python
In [82]: ( plan, fraction ) = group.compute_cartesian_path( waypoints, 0.01, 0.0 )
```

`compute_cartesian_path()` で得られた計画 `plan` を
`group.execute()` に渡してロボットで動作を実行します．

```python
In [83]: group.execute( plan )
Out[83]: True
```


#### 四角形や円に沿ってエンドエフェクタを動かす

エンドエフェクタを四角形や円に沿って動かすような場合も
複数の異なる姿勢を指示して動作計画と実行を行います．

四角形や円に沿った複数の姿勢のリストをそれぞれ
`waypoints_rectangular` と `waypoints_circular` として
用意しているのでそれらを使います．

```python
In [91]: print( waypoints_rectangular )
[position:
  x: 0.25
  y: 0.0
  z: 0.1
orientation:
  x: 0.0
  y: -0.707106781187
  z: 0.0
  w: 0.707106781187, position:
  x: 0.25
  y: -0.2
  z: 0.1
orientation:
  x: 0.0
  y: -0.707106781187
  z: 0.0
  w: 0.707106781187, position:
  x: 0.45
  y: -0.2
  z: 0.1
orientation:
  x: 0.0
  y: -0.707106781187
  z: 0.0
  w: 0.707106781187, position:
  x: 0.45
  y: 0.0
  z: 0.1
orientation:
  x: 0.0
  y: -0.707106781187
  z: 0.0
  w: 0.707106781187, position:
  x: 0.25
  y: 0.0
  z: 0.1
orientation:
  x: 0.0
  y: -0.707106781187
  z: 0.0
  w: 0.707106781187]

In [92]: ( plan, fraction ) = group.compute_cartesian_path( waypoints_rectangular, 0.01, 0.0 )

In [93]: group.execute( plan )
Out[93]: True
```

同様に円に沿った動作を行います．

```python
In [94]: ( plan, fraction ) = group.compute_cartesian_path( waypoints_circular, 0.01, 0.0 )

In [95]: group.execute( plan )
Out[95]: True
```

`exit` もしくは `quit` で終了します．

```python
In [96]: exit
```

<$ifeq <$ROS_DISTRO>|indigo>

- 注意: Indigoでは
```
terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
Aborted (core dumped)
```
というエラーが表示されます．このままでも動作に支障はありませんが気になる方は以下のようにすることで正常終了できます．
```
In [97]: import moveit_commander
In [98]: moveit_commander.roscpp_shutdown()
In [99]: moveit_commander.os._exit(0)
```

<$endif>

<$endif>

<$if <$ROS_DISTRO>==melodic|<$ROS_DISTRO>==noetic>

### myCobot の場合

「シミュレータと MoveIt! の起動」の
「[myCobot - MoveIt! シミュレータ](moveit-tutorial_ja_robot-simulator.md#start-mycobot-moveit-simulator)」を参照して
MoveIt! を起動します．

次に1行もしくは複数行ごとにプログラミングとその実行を行う
対話的プログラミングコンソールを起動します．

**ターミナル** : 対話的プログラミングのコンソールの起動

```bash
source /opt/ros/<$ROS_DISTRO>/setup.bash
rosrun tork_moveit_tutorial demo.py
```

#### 実機を動かす場合

先に「[実機の使い方 - myCobot の場合](moveit-tutorial_ja_how-to-use-real-mycobot.md)」を参照してください．

そのあと，上に書かれたコマンドの代わりに，
以下を実行してください．

**ターミナル-1** : myCobot 280 とパソコンの接続プログラムの起動

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch tork_moveit_tutorial mycobot_interface.launch
```

**ターミナル-2** : myCobot 280 用の MoveIt! の起動

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch tork_moveit_tutorial demo.launch mode:=real
```

**ターミナル-3** : 対話的プログラミングのコンソールの起動

```bash
source /opt/ros/<$ROS_DISTRO>/setup.bash
rosrun tork_moveit_tutorial demo.py
```

#### 特定の関節を動かす

プログラムから関節を動かすなどをするために「右腕」の `group` を作成します．
次の `In[1]:` 以下につづくプログラムを入力して [Enter/Return] キーを押して
1行ずつ実行してください．

- 補足 : `In [1]:` の `[ ]` の中の数字はプログラム入力するたびに更新されます．本チュートリアルにある入力例にある数字と一致しない場合がありますがそれは問題ありませんので，そのままチュートリアルを進めてください．

```python
In [1]: group = MoveGroupCommander("arm_group")
[ INFO] [1511506815.441893962, 135.795000000]: TrajectoryExecution will use new action capability.
[ INFO] [1511506815.442105792, 135.795000000]: Ready to take MoveGroup commands for group right_arm.

In [2]:
```

- 注意：以下のように`TrajectoryExecution will use old action capability.`と表示される場合も有りますが問題ありません．
```python
In [1]: group = MoveGroupCommander("arm_group")
[ INFO] [1511506815.441893962, 135.795000000]: TrajectoryExecution will use old action capability.
[ INFO] [1511506815.442105792, 135.795000000]: Ready to take MoveGroup commands for group right_arm.
```

グループ `group` に含まれる関節の名前を `get_joints()` で調べます．

```python
In [2]: group.get_joints()
Out[2]: 
['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

In [3]:
```

上は myCobot 280 の場合の出力結果で
`joint1`〜`joint6` の6つの関節があることがわかります．

肘関節に相当する `joint3` を動かしてみます．

`set_joint_value_target()` を使って関節の目標値を設定します．
`set_joint_value_target()` に関節名 `'joint3'` と
関節角度を `-2.0`（単位ラジアン[rad]）を渡します．

```python
In [3]: group.set_joint_value_target( 'joint3', -2.0 )
```

関節角度目標を設定したので `go()` で動かします．

```python
In [4]: group.go()
Out[4]: True
```

正常に動作が完了すると `True` が返ってきます．
肘関節 `joint3` が少し屈曲したと思います．

同じ要領で他の右腕の関節もそれぞれ動かしてみます．

```python
In [5]: group.set_joint_value_target( 'joint4', -0.78 )
In [6]: group.go()
Out[6]: True

In [7]: group.set_joint_value_target( 'joint5', 0.78 )
In [8]: group.go()
Out[8]: True

In [9]: group.set_joint_value_target( 'joint6', 0.78 )
In [10]: group.go()
Out[10]: True
```

#### 腕全体の関節を動かす

`set_joint_value_target()` は1つの関節だけでなく
腕全体の関節角度目標値のリストを渡すことで複数の関節を同時に動かすこともできます．
`group.get_joints()` で調べたように myCobot には6つの関節があるので
6つ分の関節角度目標値 [rad] を持ったリストを `set_joint_value_target()` に渡します．

```python
In [11]: group.set_joint_value_target( [ 0.0, 0.0, 0, 0.0, 0.0, 0.0 ] )
In [12]: group.go()
Out[12]: True
```

myCobot ロボットは初期姿勢に戻っていることと思います．

#### 手先の位置を指定して動かす

手先（エンドエフェクタリンク）の位置を指定して腕を動かしてみます．

`set_position_target()` を使います．
手先の目標位置の座標 ( x, y, z ) をリスト `[ X, Y, Z ]` で
`set_position_target()` に渡します．

位置座標の単位はメートル [m] です．
座標原点は myCobot の場合，ベースリンク (`/base`) にあります．
各軸の方向は下記の右手座標系です．

- X方向 : 正 = 前 / 負 = 後
- Y方向 : 正 = 左 / 負 = 右
- Z方向 : 正 = 上 / 負 = 下

```python
In [21]: group.set_position_target( [ 0.1, -0.1, 0.1 ] )
```

`go()` を使って実行します．

```python
In [22]: group.go()
Out[22]: True
```

手先の「位置」しか指定していないので
手先の「姿勢」は思わぬ方を向いていることもあります．

#### 手先の姿勢を指定して動かす

手先の姿勢を指定してロボットを動かしてみます．

`set_rpy_target()` を使います．
手先の目標姿勢の角度 ( roll, pitch, yaw ) をリスト `[ R, P, Y ]` で
`set_rpy_target()` に渡します．

```python
In [31]: group.set_rpy_target( [ 0.0, -2.36, 0.0 ] )
```

`go()` を使って実行します．

```python
In [32]: group.go()
Out[32]: True
```

手先の「姿勢」しか指定していないので
手先の「位置」は思わぬところにあることもあります．

- 注意：以下のようなエラーメッセージが表示された場合は，再度 `group.go()` をお送りください．

```bash
[ INFO] [1515668193.145149146, 166.319999999]: ABORTED: Solution found but controller failed during execution
```

#### 手先の位置と姿勢を指定して動かす

手先の位置と姿勢を同時に指定して腕を動かすことができます．

`set_pose_target()` を使います．
次のいずれか1つを `set_pose_target()` に渡して手先の位置と姿勢を指定します．

- 位置座標と Roll/Pitch/Yaw 姿勢角の6つの数値のリスト `[x, y, z, rot_x, rot_y, rot_z]`
- 位置座標とクォータニオンの7つの数値のリスト `[x, y, z, qx, qy, qz, qw]`
- `Pose`型
- `PoseStamped`型

どれを渡しても `set_pose_target()` 内で判断して適切な処理がなされます．

- 参考 : GitHub - move_group.py 内の `set_pose_target()` の定義

[https://github.com/ros-planning/moveit/blob/1.0.2/moveit_commander/src/moveit_commander/move_group.py#L252][6cf69e4b]

  [6cf69e4b]: https://github.com/ros-planning/moveit/blob/1.0.2/moveit_commander/src/moveit_commander/move_group.py#L252 "set_pose_target()"

それでは右手先の位置と姿勢を指定して腕を動かしてみます．
まずは 位置とRPY角 を `set_pose_target()` に渡して動作させます．

```python
In [41]: group.set_pose_target( [ 0.1, -0.1, 0.1, 0, -1.57, 0 ] )
In [42]: group.go()
Out[42]: True
```

もう1つ位置・姿勢を指定して腕を動かしてみます．

```python
In [43]: group.set_pose_target( [ 0.1, -0.1, 0.2, 0, -3.14, 0 ] )
In [44]: group.go()
Out[44]: True
```

今度は 位置とクォータニオン を `set_pose_target()` に渡して腕を動かしてみます．

```python
In [45]: group.set_pose_target( [ 0.1, -0.1, 0.1, 0.0, -0.707, 0.0, 0.707 ] )
In [46]: group.go()
Out[46]: True
```

```python
In [47]: group.set_pose_target( [ 0.1, -0.1, 0.2, 0.0, -1.0, 0.0, 0.0] )
In [48]: group.go()
Out[48]: True
```

> - クォータニオン（四元数/しげんすう）
>   - 空間上の姿勢を表現するための4つの成分をもつベクトル
>     - 複素数から拡張された数体系
>   - 回転の結合・補間の計算が容易
>   - Roll/Pitch/Yaw のオイラー角により姿勢を表現した場合と比べて
>     - 長所
>       - ジンバルロックという特異点がない
>       - 計算が速い
>     - 短所
>       - 直感的に分かりにくい
>   - 比較的理解しやすい Roll/Pitch/Yaw からクォータニオンに変換することも可能
>     - 3D Rotation Converter
>       - https://www.andre-gaschler.com/rotationconverter/
>     - ROS Wiki - Quaternion Basics
>       - http://wiki.ros.org/tf2/Tutorials/Quaternions
>   - ロボット以外では航空宇宙や 3D グラフィックスなどの分野で用いられる

更に `Pose` 型を `set_pose_target()` に渡して腕を動かしてみます．

```python
In [49]: pose_target_1 = Pose()

In [50]: print( pose_target_1 )
position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0

In [51]: pose_target_1.position.x = 0.1
In [52]: pose_target_1.position.y = -0.1
In [53]: pose_target_1.position.z = 0.1
In [54]: pose_target_1.orientation.x = 0.0
In [55]: pose_target_1.orientation.y = -0.707
In [56]: pose_target_1.orientation.z = 0.0
In [57]: pose_target_1.orientation.w = 0.707

In [58]: print( pose_target_1 )
position:
  x: 0.1
  y: -0.1
  z: 0.1
orientation:
  x: 0.0
  y: -0.707
  z: 0.0
  w: 0.707

In [59]: group.set_pose_target( pose_target_1 )
In [60]: group.go()
Out[60]: True
```

ポーズをもう1つ指定して腕を動かします．

```python
In [61]: pose_target_2 = Pose()

In [62]: print( pose_target_2 )
position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0

In [63]: pose_target_2.position.x = 0.1
In [64]: pose_target_2.position.y = -0.1
In [65]: pose_target_2.position.z = 0.2
In [66]: pose_target_2.orientation.y = -1.0

In [68]: print( pose_target_2 )
position:
  x: 0.1
  y: -0.1
  z: 0.2
orientation:
  x: 0.0
  y: -1.0
  z: 0.0
  w: 0.0

In [69]: group.set_pose_target( pose_target_2 )
In [70]: group.go()
Out[70]: True
```

`set_pose_target()` に渡せるもう1つの型 `PoseStamped` 型については

- 発展的なロボットプログラミング - より複雑なロボット動作計画 / 姿勢の参照座標を指定する

にて説明します．

#### 直線補間軌道でロボットを動かす

`group.plan()` や `group.go()` を用いた動作計画では動作開始姿勢と目標姿勢の間の動作は
各関節の開始角度と目標角度の間を補間した動作として計画されます．
このことは開始姿勢や目標姿勢として指定した姿勢以外の動作途中におけるエンドエフェクタの姿勢は
保証されないことを意味します．

エンドエフェクタを目標姿勢間で直線的に動作させたい場合は
`group.compute_cartesian_path()` を用いて動作計画をします．
`compute_cartesian_path()` はその名前のとおり，
直行座標（=デカルト座標: Cartesian Coordinates）における補間軌道（Path）を作成します．

`group.plan()` や `group.go()` で作成された動作計画（画像:左）と
`group.compute_cartesian_path()` で作成された動作計画（画像:右）を比較すると
次のようになります．

![MoveIt! - Trajectory Comparison / compute_cartesian_path()](images/nextage_moveit_cartesian-path_linear-trajectory_comparison.png)

`compute_cartesian_path()` の具体的な使用方法は本項に続く項目の

- 連続した指令をロボットに送る
- 四角形や円に沿ってエンドエフェクタを動かす

を通して学習します．

#### 連続した指令をロボットに送る

ロボットの複数の異なる姿勢を指示して動作計画と実行を行います．

複数の姿勢を指定した動作計画を行う場合も直線補間軌道でロボットを動かす場合と同じ 
`compute_cartesian_path()` を用います．

`compute_cartesian_path( self, waypoints, eef_step, jump_threshold, avoid_collisions = True )` 
には次のものを渡します．

- `waypoints` : エンドエフェクタが経由する姿勢のリスト
- `eef_step` : エンドエフェクタの姿勢を計算する間隔の距離
- `jump_threshold` : 軌道内の連続する点間の最大距離（`0.0` で無効）
- `avoid_collisions` : 干渉と運動学上の制約チェック（デフォルトは `True` でチェックする）

本チュートリアル手順に則って進めている場合，
myCobot 用の複数の姿勢のリスト `waypoints_mycobot` が用意されているので内容を確かめてみます．

```python
In [2]: print( waypoints_mycobot )
[position:
  x: 0.1
  y: -0.1
  z: 0.1
orientation:
  x: 0.0
  y: -0.707
  z: 0.0
  w: 0.707, position:
  x: 0.1
  y: -0.15
  z: 0.1
orientation:
  x: 0.0
  y: -0.707
  z: 0.0
  w: 0.707, position:
  x: 0.1
  y: -0.1
  z: 0.2
orientation:
  x: 0.0
  y: -1.0
  z: 0.0
  w: 0.0, position:
  x: 0.1
  y: -0.2
  z: 0.2
orientation:
  x: 0.0
  y: -1.0
  z: 0.0
  w: 0.0]
```

myCobot を特異姿勢でない姿勢にしてから，
姿勢のリスト `waypoints_mycobot` を `compute_cartesian_path()` に渡して動作計画を作成します．

```python
In [3]: group.set_joint_value_target([0, 0, -1.57, 0, 0, 0])

In [4]: group.go()
Out[4]: True

In [5]: ( plan, fraction ) = group.compute_cartesian_path( waypoints_mycobot, 0.01, 0.0)
```

`compute_cartesian_path()` で得られた計画 `plan` を
`group.execute()` に渡してロボットで動作を実行します．

```python
In [6]: group.execute(plan)
Out[6]: True
```

#### 四角形や円に沿ってエンドエフェクタを動かす

エンドエフェクタを四角形や円に沿って動かすような場合も
複数の異なる姿勢を指示して動作計画と実行を行います．

四角形や円に沿った複数の姿勢のリストをそれぞれ
`waypoints_rectangular` と `waypoints_circular` として
用意しているのでそれらを使います．

```python
In [10]: print( waypoints_mycobot_rectangular )
[position:
  x: 0.1
  y: 0.1
  z: 0.1
orientation:
  x: 0.0
  y: -1.0
  z: 0.0
  w: 6.123233995736766e-17, position:
  x: 0.1
  y: 0.15
  z: 0.1
orientation:
  x: 0.0
  y: -1.0
  z: 0.0
  w: 6.123233995736766e-17, position:
  x: 0.15
  y: 0.15
  z: 0.1
orientation:
  x: 0.0
  y: -1.0
  z: 0.0
  w: 6.123233995736766e-17, position:
  x: 0.15
  y: 0.1
  z: 0.1
orientation:
  x: 0.0
  y: -1.0
  z: 0.0
  w: 6.123233995736766e-17, position:
  x: 0.1
  y: 0.1
  z: 0.1
orientation:
  x: 0.0
  y: -1.0
  z: 0.0
  w: 6.123233995736766e-17]

In [11]: group.set_joint_value_target( [ 0.0, 0.0, -1.57, 0.0, 0.0, 0.0 ] )

In [12]: group.go()
Out[12]: True

In [13]: ( plan, fraction ) = group.compute_cartesian_path( waypoints_mycobot_rectangular, 0.01, 0.0)

In [14]: group.execute(plan)
Out[14]: True
```

同様に円に沿った動作を行います．

```python
In [13]: ( plan, fraction ) = group.compute_cartesian_path( waypoints_mycobot_circular, 0.01, 0.0)

In [14]: group.execute(plan)
Out[14]: True

```

`exit` もしくは `quit` で終了します．

```python
In [15]: exit
```

<$endif>

## プログラムファイルを実行する

コンソールでのプログラム実行は学習や各コマンドの動作確認には良いのですが，
毎回同じことを入力して実行するのは大変なので命令が書かれたプログラムファイルを実行します．

プログラムファイルを実行する前に物理シミュレータと MoveIt! を起動しておきます．


<$if <$ROS_DISTRO>==indigo||<$ROS_DISTRO>==kinetic||<$ROS_DISTRO>==melodic>

### NEXTAGE OPEN の場合

**ターミナル-1**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ roslaunch nextage_gazebo nextage_world.launch
```

- メモ : hrpsys (RTM) シミュレータでも可

**ターミナル-2**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ roslaunch nextage_moveit_config moveit_planning_execution.launch
```

動作プログラムファイルを実行します．

**ターミナル-3**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ rosrun tork_moveit_tutorial nextage_moveit_tutorial_poses.py
```

**nextage_moveit_tutorial_poses.py**

```python
#!/usr/bin/env python

from tork_moveit_tutorial import *


if __name__ == '__main__':

    init_node()

    group = MoveGroupCommander("right_arm")

    # Pose Target 1
    rospy.loginfo( "Start Pose Target 1")
    pose_target_1 = Pose()

    pose_target_1.position.x = 0.4
    pose_target_1.position.y = -0.4
    pose_target_1.position.z = 0.15
    pose_target_1.orientation.x = 0.0
    pose_target_1.orientation.y = -0.707
    pose_target_1.orientation.z = 0.0
    pose_target_1.orientation.w = 0.707

    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_1 ) )
    group.set_pose_target( pose_target_1 )
    group.go()

    # Pose Target 2
    rospy.loginfo( "Start Pose Target 2")
    pose_target_2 = Pose()

    pose_target_2.position.x = 0.3
    pose_target_2.position.y = -0.3
    pose_target_2.position.z = 0.5
    pose_target_2.orientation.y = -1.0

    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_2 ) )
    group.set_pose_target( pose_target_2 )
    group.go()

```

nextage_moveit_tutorial_poses.py で実行している内容は

- **手先の位置と姿勢を指定して動かす** - `set_pose_target()` `go()`

と基本的に同じです．
主に異なるのは下記の部分です．

- `print()` を ROS のログに出力する `rospy.loginfo()` に変更
- `rospy.loginfo()` の表示内容もどの箇所の実行ログかわかるように変更

<$endif>

<$ifeq <$ROS_DISTRO>|indigo>

### Baxter Research Robot の場合

**ターミナル-1**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ roslaunch baxter_gazebo baxter_world.launch
```

**ターミナル-2**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ rosrun baxter_tools enable_robot.py -e
$ rosrun baxter_tools tuck_arms.py -u
$ rosrun baxter_interface joint_trajectory_action_server.py
```

**ターミナル-3**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ roslaunch baxter_moveit_config baxter_grippers.launch
```

動作プログラムファイルを実行します．

**ターミナル-4**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ rosrun tork_moveit_tutorial baxter_moveit_tutorial_poses.py
```

**baxter_moveit_tutorial_poses.py**

```python
#!/usr/bin/env python

from tork_moveit_tutorial import *


if __name__ == '__main__':

    init_node()

    group = MoveGroupCommander("right_arm")

    # Pose Target 1
    rospy.loginfo( "Start Pose Target 1")
    pose_target_1 = Pose()

    pose_target_1.position.x = 0.7
    pose_target_1.position.y = -0.2
    pose_target_1.position.z = 0.1
    pose_target_1.orientation.x = 0.0
    pose_target_1.orientation.y = 1.0
    pose_target_1.orientation.z = 0.0
    pose_target_1.orientation.w = 0.0

    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_1 ) )
    group.set_pose_target( pose_target_1 )
    group.go()

    # Pose Target 2
    rospy.loginfo( "Start Pose Target 2")
    pose_target_2 = Pose()

    pose_target_2.position.x = 0.9
    pose_target_2.position.y = -0.3
    pose_target_2.position.z = 0.6
    pose_target_2.orientation.y = 0.707
    pose_target_2.orientation.w = 0.707

    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_2 ) )
    group.set_pose_target( pose_target_2 )
    group.go()

```

NEXTAGE OPEN の動作計画・動作の実行ファイルとの相違点は次のとおりです．

- ターゲットポーズの位置・姿勢を Baxter の機構に適したものに変更

<$endif>


<$if <$ROS_DISTRO>==indigo||<$ROS_DISTRO>==kinetic||<$ROS_DISTRO>==melodic>

### MINAS TRA1 の場合


**ターミナル-1**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ roslaunch tra1_bringup tra1_bringup.launch simulation:=true
```

**ターミナル-2**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ roslaunch tra1_bringup tra1_moveit.launch
```

動作プログラムファイルを実行します．

**ターミナル-3**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ rosrun tork_moveit_tutorial tra1_moveit_tutorial_poses.py
```

**tra1_moveit_tutorial_poses.py**

```python
#!/usr/bin/env python

from tork_moveit_tutorial import *


if __name__ == '__main__':

    init_node()

    group = MoveGroupCommander("manipulator")

    # Pose Target 1
    rospy.loginfo( "Start Pose Target 1")
    pose_target_1 = Pose()

    pose_target_1.position.x = 0.0
    pose_target_1.position.y = -0.6
    pose_target_1.position.z = 0.3
    pose_target_1.orientation.x = 1.0
    pose_target_1.orientation.y = 0.0
    pose_target_1.orientation.z = 0.0
    pose_target_1.orientation.w = 0.0

    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_1 ) )
    group.set_pose_target( pose_target_1 )
    group.go()

    # Pose Target 2
    rospy.loginfo( "Start Pose Target 2")
    pose_target_2 = Pose()

    pose_target_2.position.x = 0.6
    pose_target_2.position.y = 0.0
    pose_target_2.position.z = 0.3
    pose_target_2.orientation.x = -0.707
    pose_target_2.orientation.y = -0.707

    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_2 ) )
    group.set_pose_target( pose_target_2 )
    group.go()

```

他のロボットの動作計画・動作の実行ファイルとの相違点は次のとおりです．

- `group = MoveGroupCommander()` に渡すグループ名を `"manipulator"` に変更
- ターゲットポーズの位置・姿勢を MINAS TRA1 の機構に適したものに変更

<$endif>


<$if <$ROS_DISTRO>==kinetic||<$ROS_DISTRO>==melodic||<$ROS_DISTRO>==noetic>

### KHI duaro の場合

**ターミナル-1**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ roslaunch khi_duaro_gazebo duaro_world.launch
```

**ターミナル-2**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ roslaunch khi_duaro_moveit_config moveit_planning_execution.launch
```

動作プログラムファイルを実行します．

**ターミナル-3**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ rosrun tork_moveit_tutorial duaro_moveit_tutorial_poses.py
```

**duaro_moveit_tutorial_poses.py**

```python
#!/usr/bin/env python

from tork_moveit_tutorial import *


if __name__ == '__main__':

    init_node()

    group = MoveGroupCommander("upper_arm")

    # Pose Target 1
    rospy.loginfo( "Start Pose Target 1")
    pose_target_1 = Pose()

    pose_target_1.position.x =  0.0
    pose_target_1.position.y =  0.55
    pose_target_1.position.z =  1.0
    pose_target_1.orientation.x =  0.0
    pose_target_1.orientation.y =  0.0
    pose_target_1.orientation.z =  0.0
    pose_target_1.orientation.w =  0.0

    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_1 ) )
    group.set_pose_target( pose_target_1 )
    group.go()

    # Pose Target 2
    rospy.loginfo( "Start Pose Target 2")
    pose_target_2 = Pose()

    pose_target_2.position.x = -0.55
    pose_target_2.position.y = -0.0
    pose_target_2.position.z =  1.05
    pose_target_2.orientation.x =  0.0
    pose_target_2.orientation.y =  0.0
    pose_target_2.orientation.z =  0.707
    pose_target_2.orientation.w =  0.707

    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_2 ) )
    group.set_pose_target( pose_target_2 )
    group.go()

```

他のロボットの動作計画・動作の実行ファイルとの相違点は次のとおりです．

- `group = MoveGroupCommander()` に渡すグループ名を `"upper_arm"` に変更
- ターゲットポーズの位置・姿勢を KHI duaro の機構に適したものに変更

<$endif>


<$if <$ROS_DISTRO>==melodic|<$ROS_DISTRO>==noetic>

### myCobot の場合

**ターミナル-1**
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch tork_moveit_tutorial demo.launch
```

動作プログラムファイルを実行します．

**ターミナル-2**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ rosrun tork_moveit_tutorial mycobot_moveit_tutorial_poses.py
```

**実機を動かす場合**

先に「[実機の使い方 - myCobot の場合](moveit-tutorial_ja_how-to-use-real-mycobot.md)」を参照してください．

そのあと，上に書かれたコマンドの代わりに，
以下を実行してください．

**ターミナル-1** : myCobot 280 とパソコンの接続プログラムの起動

```bash
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch tork_moveit_tutorial mycobot_interface.launch
```

**ターミナル-2** : myCobot 280 用の MoveIt! の起動

```bash
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch tork_moveit_tutorial demo.launch mode:=real
```

**ターミナル-3**
```
$ source /opt/ros/<$ROS_DISTRO>/setup.bash
$ rosrun tork_moveit_tutorial mycobot_moveit_tutorial_poses.py
```

`set_pose_target` を用いて手先位置姿勢を指定し，腕を動かすプログラムの例が以下になります．

**mycobot_moveit_tutorial_poses.py**

```python
#!/usr/bin/env python

from tork_moveit_tutorial import *
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':

    init_node()

    group = MoveGroupCommander("arm_group")
    # Pose Target 1
    rospy.loginfo( "Start Pose Target 1")
    pose_target_1 = Pose()

    # quaternion_from_euler(0, 0, -1.57079)
    pose_target_1.position.x = 0.2
    pose_target_1.position.y = 0.0
    pose_target_1.position.z = 0.2
    pose_target_1.orientation.x = 0.0
    pose_target_1.orientation.y = 0.0
    pose_target_1.orientation.z = -0.7071
    pose_target_1.orientation.w =  0.7071

    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_1 ) )
    group.set_pose_target( pose_target_1 )
    group.go()

    # Pose Target 2
    rospy.loginfo( "Start Pose Target 2")
    pose_target_2 = Pose()

    pose_target_2.position.x = 0.0
    pose_target_2.position.y =-0.2
    pose_target_2.position.z = 0.2
    pose_target_2.orientation.z = -0.7071
    pose_target_2.orientation.w =  0.7071

    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_2 ) )
    group.set_pose_target( pose_target_2 )
    group.go()

    # Compute Cartesian path
    (plan, fraction) = group.compute_cartesian_path([pose_target_1, pose_target_2], 0.01, 0.0)
    group.execute( plan )
```

![myCobot MoveIt! - cartesian path](images/melodic/mycobot-moveit_pose.png)

他のロボットの動作計画・動作の実行ファイルとの相違点は次のとおりです．

- `group = MoveGroupCommander()` に渡すグループ名を `"arm_group"` に変更
- ターゲットポーズの位置・姿勢を myCobot の機構に適したものに変更

`Pose()` に指定する姿勢情報のクオータニオン情報は，
`from tf.transformations import quaternion_from_euler`
で得られる，`quaternion_from_euler(0, 0, -1.57079)` 関数を用いて取得します．

また，このプログラムに続けて
以下のように，`pose_target_1`, `pose_target_2`を用いて
`group.compute_cartesian_path([pose_target_1, pose_target_2], 0.01, 0.0)`
として直線補間軌道を計画し，`group.execute( plan )` で実行します．

```
    # Compute Cartesian path
    (plan, fraction) = group.compute_cartesian_path([pose_target_1, pose_target_2], 0.01, 0.0)
    group.execute( plan )
```



![myCobot MoveIt! - cartesian path](images/melodic/mycobot-moveit_cartesian_path.png)

<$endif>


### ROS や MoveIt! のメリット

<$if <$ROS_DISTRO>==indigo>

NEXTAGE OPEN や Baxter Research Robot, MINAS TRA1
の動作計画・動作プログラムの相違点を見ると下記の2ヶ所だけが
各ロボットに対応しただけだということが分かります．

<$elif <$ROS_DISTRO>==kinetic|<$ROS_DISTRO>==melodic>

NEXTAGE OPEN や MINAS TRA1，KHI duaro
の動作計画・動作プログラムの相違点を見ると下記の2ヶ所だけが
各ロボットに対応しただけだということが分かります．

<$elif <$ROS_DISTRO>==noetic>

KHI duaro や myCobot
の動作計画・動作プログラムの相違点を見ると下記の2ヶ所だけが
各ロボットに対応しただけだということが分かります．

<$endif>

- group = MoveGroupCommander() に渡すグループ名を各ロボットアームに対応したものにする
- ターゲットポーズの位置・姿勢を各ロボットの機構に適したものにする

このように「**ロボットが異なっても基本的には同じプログラムが動く**」ということが
ROS や MoveIt! のインタフェースを使用する最大のメリットの1つであるといえます．


<!-- EOF -->
