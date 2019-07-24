
# 独自プログラムの実行

ここまでのチュートリアルでは，基本的には既存のプログラムパッケージやファイルを利用して進めてきました．

本章は Baxter Research Roobt を例に次のような作業を進めていくことで
段階的にユーザ独自のプログラムの作成・実行ができるようになることを目的としています．

- ワークスペースの作成
- ソースコードの取得とビルド
    - ソースコードの取得（=クローンなど）
    - ソースコードを実行できるようにする（=ビルド）
- ビルドしたソフトウェアの実行
- ユーザ独自プログラムの作成と実行
    - 既存のプログラムの変更
    - 新規プログラムの追加


## ワークスペースの作成

ROS のプログラム開発作業ディレクトリを「ワークスペース」といいます．
独自プログラムの開発はワークスペースで行います．

- catkin の workspace を作る
  - http://wiki.ros.org/ja/catkin/Tutorials/create_a_workspace

`catkin_ws` という名前のワークスペースを作成する手順は次のとおりです．

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```

上記の `$ source devel/setup.bash` のコマンドは
ROS からワークスペースへの参照パスを設定しています．
これはターミナルを開くたびに

これはワークスペースを利用する場合は新しくターミナルを立ち上げて 
ROS を使用する前に毎回必要になります．
下記のように .bashrc ファイルに設定を加えて
ターミナル起動時に setup.bash を自動で実行して
ワークスペースを利用した ROS 環境になるようにすると便利です．

```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

- **注意** : 上記コマンドの `>>` を `>` にしてしまうと
  元々あった .bashrc 内の設定が消えてしまうので気をつけてください．

.bashrc の設定ができていると以後のターミナルを起動するたびに行う
`source ~/catkin_ws/devel/setup.bash` は不要です．

ROS 環境の確認は次のコマンドを入力して行うことができます．

```
$ env | grep ROS
```

下記は ROS 環境を表示した例です．
`ROS_PACKAGE_PATH` にワークスペースのディレクトリ 
`/home/robotuser/catkin_ws/src` があることで，
ROS のシステムからワークスペース内のパッケージにある実行ファイルなどを
参照することができるようになっています．

```
robotuser@robotuser-PC:~/catkin_ws$ env | grep ROS
ROS_ROOT=/opt/ros/kinetic/share/ros
ROS_PACKAGE_PATH=/home/robotuser/catkin_ws/src:/opt/ros/kinetic/share
ROS_MASTER_URI=http://localhost:11311
ROS_VERSION=1
ROSLISP_PACKAGE_DIRECTORIES=/home/robotuser/catkin_ws/devel/share/common-lisp
ROS_DISTRO=kinetic
ROS_ETC_DIR=/opt/ros/kinetic/etc/ros
```

> **ワークスペース名について**  
> 
> ワークスペース名は基本的に半角英数字であれば何でも大丈夫です．
> 
> 本チュートリアルに限らず多くのチュートリアルでは 
> `catkin_ws` や `ros_ws` といったワークスペース名が多く使われています．
> しかし，いろいろな種類のロボットやセンサなどのプログラムを書いていると
> ワークスペースを分けたくなることもあります．
> 
> 発展的に独自プログラムを作成する際に新たにワークペースを追加する場合は
> 他のワークスペース名と重複しないワークスペース名をつけてください．
> 
> 例えば，上で `catkin_ws` としたところを 
> Baxter Research Robot を扱うということで
> `baxter_ws` や `brr_ws` といったワークスペース名としてみるのも良いでしょう．


## ソースコードの取得とビルド

前節で作成したワークスペースに既存の ROS パッケージをネットワークを介して 
Ubuntu PC 上のワークスペースにコピー（クローン）して，
それをビルドすることでプログラムを実行できる状態にします．

クローンとビルドを行う ROS パッケージは 
Baxter Research Robot の下記のパッケージです．

- Baxter Research Robot
    - Software Development Kit (SDK)
    - Baxter MoveIt! Configuration Package
    - Baxter Simulator

### Baxter Software Development Kit の取得とビルド

次の手順で Baxter Software Development Kit のクローンと
それに必要なソフトウェアパッケージの取得，ビルドを行います．

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/RethinkRobotics/baxter.git
$ cd ~/catkin_ws/src/baxter
$ git checkout master
$ cd ~/catkin_ws
$ wstool merge -t src ~/catkin_ws/src/baxter/baxter_sdk.rosinstall
$ wstool update -t src
$ rosdep install -y -r --from-paths src --ignore-src
$ catkin_make
$ source devel/setup.bash
```

### Baxter MoveIt! Configuration の取得とビルド

次の手順で Baxter MoveIt! Configuration のクローンと
それに必要なソフトウェアパッケージの取得，ビルドを行います．

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-planning/moveit_robots.git
$ cd ~/catkin_ws/src/moveit_robots
$ git checkout <$ROS_DISTRO>-devel
$ cd ~/catkin_ws
$ rosdep install -y -r --from-paths src --ignore-src
$ catkin_make
$ source devel/setup.bash
```

### Baxter Simulator の取得とビルド

次の手順で Baxter Simulator のクローンと
それに必要なソフトウェアパッケージの取得，ビルドを行います．

```
$ sudo apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser
$ cd ~/catkin_ws
$ wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/kinetic-devel/baxter_simulator.rosinstall
$ wstool update
$ rosdep install -y -r --from-paths src --ignore-src
$ catkin_make
$ source devel/setup.bash
```

## ビルドしたソフトウェアの実行

前節のソフトウェアコードの取得とビルドがエラーなく進んでいれば
この時点でそれらのソフトウェアを実行できる状態になっているはずです．

ビルドできていれば基本的に `sudo apt-get install ...` でインストールした 
ROS パッケージと同じ要領でソフトウェアを実行することができます．
唯一違うのはソフトウェアをビルドしたワークスペースにも 
ROS 環境のパッケージパスを通すという点です．

```
$ source ~/catkin_ws/devel/setup.bash
```

これはワークスペースの作成のところで記述したように
ターミナルを起動するたびに行われる必要があります．
面倒な場合は .bashrc ファイルを編集して
ターミナルを起動するたびに自動的に実行されるようにしてください．
（参照: ワークスペースの作成）

Baxter Simulator 上の Baxter ロボットを MoveIt! から動かすために
ターミナルを3つ開き，それぞれで次のことを行います．

- ターミナル-1 : Baxter Gazebo シミュレータの実行
- ターミナル-2 : Baxter ロボットの初期化とインタフェースの起動
- ターミナル-3 : Baxter MoveIt! の実行

### Gazebo シミュレータの実行

Baxter Research Robot の Gazebo シミュレータを起動します．
1つ目のターミナル上で次のコマンドを実行してください．


**ターミナル-1** : Baxter Gazebo シミュレータの実行
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch baxter_gazebo baxter_world.launch  
```

<$ifeq <$ROS_DISTRO>|kinetic>

![Baxter Simulator - Starts](images/kinetic/baxter-simulator_starts.png)

<$endif>

しばらくすると次のようなメッセージが **ターミナル-1** に表示されます．

```
[ INFO] [1509004453.402952141, 10.130000000]: Simulator is loaded and started successfully
[ INFO] [1509004453.462744480, 10.140000000]: Robot is disabled
[ INFO] [1509004453.462870807, 10.140000000]: Gravity compensation was turned off
```

これで Gazebo シミュレータの起動は完了した状態になっています．

### ロボットの初期化とインタフェースの起動

次に Baxter Research Robot を MoveIt! から操作可能な状態になるように準備を行います．
2つ目のターミナル上で次のコマンドを実行してください．

**ターミナル-2** : ロボットの初期化とインタフェースの起動
```
$ source ~/catkin_ws/devel/setup.bash
$ rosrun baxter_tools enable_robot.py -e

[INFO] [WallTime: 1509004993.657452] [192.385000] Robot Enabled

$ rosrun baxter_tools tuck_arms.py -u

[INFO] [WallTime: 1509005020.475291] [0.000000] Untucking arms
[INFO] [WallTime: 1509005020.910480] [202.807000] Moving head to neutral position
[INFO] [WallTime: 1509005020.911446] [202.808000] Untucking: Arms already Untucked; Moving to neutral position.
[INFO] [WallTime: 1509005024.476011] [204.036000] Finished tuck

$ rosrun baxter_interface joint_trajectory_action_server.py

Initializing node...
Initializing joint trajectory action server...
Running. Ctrl-c to quit
```

<$ifeq <$ROS_DISTRO>|kinetic>

![Baxter Simulator - Ready for MoveIt!](images/kinetic/baxter-simulator_ready-for-moveit.png)

<$endif>

これでロボットの準備は完了です．

### MoveIt! の起動

3つ目のターミナルで次のコマンドを実行して MoveIt! を起動します．

**ターミナル-3** : MoveIt! の起動
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch baxter_moveit_config baxter_grippers.launch
```

<$ifeq <$ROS_DISTRO>|kinetic>

![Baxter MoveIt! - Starts](images/kinetic/baxter-moveit_starts.png)

<$endif>

これで MoveIt! の動作計画機能が利用できる状態になっています．

これまでのチュートリアルで行ったのと同様に MoveIt! の GUI から動作生成と実行を試してみてください．


## ユーザ独自プログラム作成と実行

これまでは

### チュートリアルソースコードの取得とビルド

次の手順で TORK MoveIt! Tutorial パッケージのクローンと
それに必要なソフトウェアパッケージの取得，ビルドを行います．

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/tork-a/tork_moveit_tutorial.git
$ cd ~/catkin_ws
$ rosdep install -y -r --from-paths src --ignore-src
$ catkin_make
$ source devel/setup.bash
```

### ROS 環境の確認

前項のプログラム TORK MoveIt! Tutorial パッケージは

```
$ env | grep ROS
```

ROS 環境確認結果の例

```
robotuser@robotuser-PC:~/catkin_ws$ env | grep ROS
ROS_ROOT=/opt/ros/kinetic/share/ros
ROS_PACKAGE_PATH=/home/robotuser/catkin_ws/src:/opt/ros/kinetic/share
ROS_MASTER_URI=http://localhost:11311
ROS_VERSION=1
ROSLISP_PACKAGE_DIRECTORIES=/home/robotuser/catkin_ws/devel/share/common-lisp
ROS_DISTRO=kinetic
ROS_ETC_DIR=/opt/ros/kinetic/etc/ros
```

```
$ rospack find tork_movieit_tutorial
```

ワークスペースの `tork_moveit_tutorial` が参照されている場合

```
robotuser@robotuser-PC:~/catkin_ws$ rospack find tork_moveit_tutorial 
/home/robotuser/catkin_ws/src/tork_moveit_tutorial
```

ワークスペースの `tork_moveit_tutorial` が参照されて **いない** 場合

```
robotuser@robotuser-PC:~/catkin_ws$ rospack find tork_moveit_tutorial 
/opt/ros/kinetic/share/tork_moveit_tutorial
```

もう一度

```
$ source ~/catkin_ws/devel/setup.bash
```

OKだったら

シミュレーションの実行 
ロボットの準備
MoveIt! の起動
プログラムの実行

```
$ rosrun tork_moveit_tutorial baxter_moveit_tutorial_poses.py
```


    - 既存のプログラムの変更
    - 新規プログラムの追加

### 既存のプログラムの変更

### プログラムの実行

### 新しいプログラムファイルの作成



???.py という名前をつけましたが普通のテキストファイルなので
そのままだとプログラムファイルとして実行できない状態にあります．
プログラムファイルとして実行できるようにするためには
ファイルの実行権限設定を行う必要があります．


<!--
## 複数のプログラムの実行

### launch ファイルの作成

本チュートリアルの演習を行っていると既に実行したことがあるかと思います．

例えば下記のような launch ファイルです．

```

```

launch ファイルではプログラムファイルに行った実行権限の設定は必要ありません


## 独自パッケージの作成

既存のパッケージに実行ファイルや launch ファイルを付け加えるのではなく，
-->

