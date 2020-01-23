
# クラス・関数リファレンス

## MoveIt! Commander

### MoveGroupCommander クラス

GitHub - [moveit_commander/src/moveit_commander/move_group.py][67977aee]

GitHub - [moveit/moveit_ros/planning_interface/move_group_interface/src/wrap_python_move_group.cpp][7a957dfd]

  [67977aee]: https://github.com/ros-planning/moveit_commander/blob/<$ROS_DISTRO>-devel/src/moveit_commander/move_group.py "move_group.py"
  [7a957dfd]: https://github.com/ros-planning/moveit/blob/<$ROS_DISTRO>-devel/moveit_ros/planning_interface/move_group_interface/src/wrap_python_move_group.cpp "wrap_python_move_group.cpp"

---
#### def \_\_init\_\_(self, name, robot_description="robot_description"):

- 機能
  - コンストラクタ
- 引数
  - `name` - str : グループ名
  - `robot_description` - str : ロボット名
- 戻り値
  - なし

---
#### def get_name(self):

- 機能
  - グループ名の取得
- 引数
  - なし
- 戻り値
  - str : グループ名

---
#### stop(self):

- 機能
  - 実行の停止
- 引数
  - なし
- 戻り値
  - なし

---
#### def get_active_joints(self):

- 機能
  - グループの機能している関節名の取得
- 引数
  - なし
- 戻り値
  - str : 機能している関節名のリスト

---
#### def get_joints(self):

- 機能
  - グループ内の関節名の取得
- 引数
  - なし
- 戻り値
  - str : 関節名のリスト

---
#### def get_variable_count(self):

- 機能
  - グループの状態をパラメータ化するために使用される変数の数を返す（DOFの数より大きいか等しい）
- 引数
  - なし
- 戻り値
  - int : 変数の数

---
#### has_end_effector_link(self):

- 機能
  - グループのエンドエフェクタリンクの有無のチェック
- 引数
  - なし
- 戻り値
  - bool : 有無

---
#### def get_end_effector_link(self):

- 機能
  - エンドエフェクタリンク名の取得
- 引数
  - なし
- 戻り値
  - str : リンク名（ 空の文字列ならエンドエフェクタリンク設定無し ）

---
#### def set_end_effector_link(self, link_name):

- 機能
  - エンドエフェクタとするリンク名の設定
- 引数
  - `link_name` - str : リンク名
- 戻り値
  - なし

---
#### def get_pose_reference_frame(self):

- 機能
  - エンドエフェクタの姿勢定義参照リンク名の取得
- 引数
  - なし
- 戻り値
  - str : リンク名

---
#### def set_pose_reference_frame(self, reference_frame):

- 機能
  - エンドエフェクタの姿勢定義参照リンク名の設定
- 引数
  - `link_name` - str : リンク名
- 戻り値
  - なし

---
#### def get_planning_frame(self):

- 機能
  - 動作計画で使用されるフレーム名の取得
- 引数
  - なし
- 戻り値
  - str : リンク名

---
#### def get_current_joint_values(self):

- 機能
  - 現在の関節角度値リストの取得
- 引数
  - なし
- 戻り値
  - [ float, float, float, ... ] : 関節角度値のリスト

---
#### def get_current_pose(self, end_effector_link = ""):

- 機能
  - グループのエンドエフェクタの姿勢の取得
- 引数
  - `end_effector_link` - str : エンドエフェクタの名前
- 戻り値
  - PoseStamped : エンドエフェクタの姿勢

---
#### def get_current_rpy(self, end_effector_link = ""):

- 機能
  - グループのエンドエフェクタの Roll, Pitch, Yaw で定義される姿勢の取得
- 引数
  - `end_effector_link` - str : エンドエフェクタの名前
- 戻り値
  - [ float, float, float ] : エンドエフェクタの姿勢 [ Roll, Pitch, Yaw ] [rad]

---
#### def get_random_joint_values(self):

- 機能
  - ランダムな関節角度値の取得
- 引数
  - なし
- 戻り値
  - [ float, float, float, ... ] : 関節角度値のリスト

---
#### def get_random_pose(self, end_effector_link = ""):

- 機能
  - ランダムなエンドエフェクタ姿勢の取得
- 引数
  - `end_effector_link` - str : エンドエフェクタの名前
- 戻り値
  - PoseStamped : エンドエフェクタの姿勢

---
#### def set_start_state_to_current_state(self):

- 機能
  - ロボットの現在の状態を動作計画の開始姿勢に設定
- 引数
  - なし
- 戻り値
  - なし

---
#### def set_start_state(self, msg):

- 機能
  - ロボットの現在の状態の代わりに異なる開始状態の設定
- 引数
  - `msg` - RobotState : ロボットの状態（ moveit_msgs/RobotState.msg ）
- 戻り値
  - なし

---
#### def set_joint_value_target(self, arg1, arg2 = None, arg3 = None):

- 機能
  - グループの関節角度目標値の設定
- 引数
  - `arg1` - dict or list or JointState : 関節角度目標値データセット [rad]
  	- `arg2`, `arg3` = None : 設定の必要なし
  - `arg1` - string : 特定のジョイント名
	  - `arg2` - float or [ float, float, ... ] : 特定関節の角度目標値 [rad]
  - `arg1` - Pose or PoseStamped
  	- `arg2`, `arg3` - str or bool
		  - str : 姿勢が指定されたエンドエフェクタ
			- bool : 指定された姿勢が近似値か否かを決定 / デフォルト False（厳密解）
    - 逆運動学計算を行いグループの関節角度目標値を設定
- 戻り値
  - なし

---
#### def set_rpy_target(self, rpy, end_effector_link = ""):

- 機能
  - エンドエフェクタの目標姿勢 Roll, Pitch, Yaw 角度を設定
- 引数
  - `rpy` - [ float, float, float ] : 目標姿勢角度 Roll, Pitch, Yaw [rad]
  - `end_effector_link` - str : エンドエフェクタの名前
- 戻り値
  - なし

---
#### def set_orientation_target(self, q, end_effector_link = ""):

- 機能
  - エンドエフェクタの目標姿勢クォータニオンを設定
- 引数
  - `q` - [ float, float, float, float ] : 目標姿勢クオータニオン
  - `end_effector_link` - str : エンドエフェクタの名前
- 戻り値
  - なし

---
#### def set_position_target(self, xyz, end_effector_link = ""):

- 機能
  - エンドエフェクタの目標位置を設定
- 引数
  - `xyz` - [ float, float, float ] : 目標位置 X, Y, Z [m]
  - `end_effector_link` - str : エンドエフェクタの名前
- 戻り値
  - なし

---
#### def set_pose_target(self, pose, end_effector_link = ""):

- 機能
  - エンドエフェクタの目標姿勢（位置・方向）の設定
- 引数
  - `pose` : 次のいずれか
    - [x, y, z, rot_x, rot_y, rot_z] : 位置座標と Roll/Pitch/Yaw 姿勢角の6つの数値のリスト
    - [x, y, z, qx, qy, qz, qw] : 位置座標とクォータニオンの7つの数値のリスト
    - Pose
    - PoseStamped
  - `end_effector_link` - str : エンドエフェクタ名 / デフォルト ""
- 戻り値
  - なし

---
#### def set_pose_targets(self, poses, end_effector_link = ""):

- 機能
  - エンドエフェクタの複数の目標姿勢（位置・方向）の設定
- 引数
  - `poses` - [ Pose, Pose, Pose, ... ] : 目標姿勢 Pose のリスト
  - `end_effector_link` - str : エンドエフェクタ名 / デフォルト ""
- 戻り値
  - なし

---
#### def shift_pose_target(self, axis, value, end_effector_link = ""):

- 機能
  - エンドエフェクタの現在の姿勢を取得し，対応する軸に値を加えてそれを目標姿勢として設定
- 引数
  - `axis` - int : 0〜5 がそれぞれ X, Y, Z, R, P, Y 軸に対応
  - `value` - float : 加算する値 [m] or [rad]
  - `end_effector_link` - str : エンドエフェクタ名 / デフォルト ""
- 戻り値
  - なし

---
#### def clear_pose_target(self, end_effector_link):

- 機能
  - 指定エンドエフェクタの目標姿勢の消去
- 引数
  - `end_effector_link` - str : エンドエフェクタ名
- 戻り値
  - なし

---
#### def clear_pose_targets(self):

- 機能
  - 全ての目標姿勢の消去
- 引数
  - なし
- 戻り値
  - なし

---
#### def set_random_target(self):

- 機能
  - ランダムな関節角度目標値の設定
- 引数
  - なし
- 戻り値
  - なし

---
#### def set_named_target(self, name):

- 機能
  - 下記により名称が設定されている関節角度目標値の設定
    - remember_joint_values() で記録
    - SRDF で設定
- 引数
  - `name` - str : 関節角度値の名称
- 戻り値
  - なし

---
#### def remember_joint_values(self, name, values = None):

- 機能
  - グループの関節角度値構成を名前をつけて記録
- 引数
  - `name` - str : 関節角度値構成の記録名称
  - `values` - [ float, float, float, ... ] : 関節角度値のリスト
	  / デフォルト None → get_current_joint_values() で現在の関節角度値を取得
- 戻り値
  - なし

---
#### def get_remembered_joint_values(self):

- 機能
  - グループで記録されている関節角度値のディクショナリを取得
- 引数
  - なし
- 戻り値
  - dict : 関節角度値構成のディクショナリ

---
#### def forget_joint_values(self, name):

- 機能
  - 指定した名称の関節角度値の破棄
- 引数
  - `name` - str : 関節角度値の名称
- 戻り値
  - なし

---
#### def get_goal_tolerance(self):

- 機能
  - 関節角度，位置，方向の目標に対する許容値の取得
- 引数
  - なし
- 戻り値
  - tuple : 関節角度，位置，姿勢の目標に対する許容値

---
#### def get_goal_joint_tolerance(self):

- 機能
  - 関節角度の目標に対する許容値の取得
- 引数
  - なし
- 戻り値
  - float : 関節角度の目標に対する許容値

---
####  def get_goal_position_tolerance(self):

- 機能
  - エンドエフェクタ位置の目標に対する許容値の取得
	  - 許容値は目標位置を中心とした球として設定
- 引数
  - なし
- 戻り値
  - float : 位置の目標に対する許容値

---
#### def get_goal_orientation_tolerance(self):

- 機能
  - エンドエフェクタの姿勢（方向）の目標に対する許容値の取得
- 引数
  - なし
- 戻り値
  - float : 姿勢（方向）の目標に対する許容値

---
#### def set_goal_tolerance(self, value):

- 機能
  - 関節角度，位置および方向の目標に対する許容値を同時に設定
- 引数
  - `value` - float : 関節角度，位置および方向の目標に対する許容値
- 戻り値
  - なし

---
#### def set_goal_joint_tolerance(self, value):

- 機能
  - 関節角度の目標に対する許容値を設定
- 引数
  - `value` - float : 関節角度の目標に対する許容値
- 戻り値
  - なし

---
#### def set_goal_position_tolerance(self, value):

- 機能
  - 位置の目標に対する許容値を設定
- 引数
  - `value` - float : 位置の目標に対する許容値
- 戻り値
  - なし

---
#### def set_goal_orientation_tolerance(self, value):

- 機能
  - 方向の目標に対する許容値を設定
- 引数
  - `value` - float : 方向の目標に対する許容値
- 戻り値
  - なし

---
#### def allow_looking(self, value):

- 機能
  - ロボットが移動する前に見回すことができるかどうかを指定
- 引数
  - `value` - bool : 見回すか否か / デフォルト True
- 戻り値
  - なし

---
#### def allow_replanning(self, value):

- 機能
  - 環境の変化を検出したときに動作の再計画をするか否かを指定
- 引数
  - `value` - bool : 再計画するか否か
- 戻り値
  - なし

---
#### def get_known_constraints(self):

- 機能
  - このグループに固有の拘束条件の名前のリストをデータベースから取得
- 引数
  - なし
- 戻り値
  - [ str, str, ... ] : 名前のリスト

---
#### def get_path_constraints(self):

- 機能
  - moveit_msgs.msgs.Constraints 形式で実際の軌道拘束条件を取得
- 引数
  - なし
- 戻り値
  - Constraints : 軌道拘束条件

---
#### def set_path_constraints(self, value):

- 機能
  - 軌道拘束条件の設定
- 引数
  - `value` - Constraints : 軌道拘束条件
- 戻り値
  - なし

---
#### def clear_path_constraints(self):

- 機能
  - 軌道拘束条件の消去
- 引数
  - なし
- 戻り値
  - なし

---
#### def set_constraints_database(self, host, port):

- 機能
  - 既知の拘束条件を保持しているデータベースサーバが存在する場所を設定
- 引数
  - `host` - string : ホスト名
	- `port` - int : ポート番号
- 戻り値
  - なし

---
#### def set_planning_time(self, seconds):

- 機能
  - 動作計画に使用する時間の設定
- 引数
  - `seconds` - float : 時間 [s]
- 戻り値
  - なし

---
#### def get_planning_time(self):

- 機能
  - 動作計画に使用する時間の取得
- 引数
  - なし
- 戻り値
  - float : 時間 [s]

---
#### def set_planner_id(self, planner_id):

- 機能
  - 動作計画に使用するプランナの設定
- 引数
  - `planner_id` - str : プランナの名称
- 戻り値
  - なし

---
#### def set_num_planning_attempts(self, num_planning_attempts):

- 機能
  - 最短解が返される前に動作計画を計算する回数の設定
- 引数
  - `num_planning_attempts` - int : 計算回数 / デフォルト 1
- 戻り値
  - なし

---
#### def set_workspace(self, ws):

- 機能
  - ロボットのワークスペースの設定
- 引数
  - `ws` - [ float, float, ...] : ワークスペース
    - [minX, minY, maxX, maxY] or
    - [minX, minY, minZ, maxX, maxY, maxZ]
- 戻り値
 - なし

---
#### def set_max_velocity_scaling_factor(self, value):

- 機能
  - 最大関節角速度減少係数の設定
- 引数
  - `value` - float : 速度減少係数 0.0〜1.0
- 戻り値
  - なし


---
#### def go(self, joints = None, wait = True):

- 機能
  - 目標を設定してグループを動かす
- 引数
  - `joints` - bool, JointState, Pose : グループの動作目標
	  （ bool の場合は wait に代入 )
		 / デフォルト None
	- `wait` - bool : 動作の終了を待つか否か / デフォルト True
- 戻り値
  - MoveItErroeCode : MoveIt! エラーコード

---
#### def plan(self, joints = None):

- 機能
  - 目標を設定し，動作計画を計算して返す
- 引数
  - `joints` - JointState, Pose, str, [ float, float, ... ] : グループの動作目標
	  / デフォルト None
- 戻り値
  - RobotTrajectory : 動作計画軌道

---
#### def compute_cartesian_path(self, waypoints, eef_step, jump_threshold, avoid_collisions = True):

- 機能
  - ウェイポイントとして定義された複数の経路上の位置・姿勢に向かってエンドエフェクタを線形補間にて動作させる軌道を作成して返す
- 引数
  - `waypoints` - [ Pose, Pose, ... ] : エンドエフェクタが経由する姿勢のリスト
  - `eef_step` - float : エンドエフェクタの姿勢を計算する間隔の距離
  - `jump_threshold` - float : 軌道内の連続する点間の最大距離（`0.0` で無効）
  - `avoid_collisions` bool : 干渉と運動学上の制約チェック（デフォルトは `True` でチェックする）
- 戻り値
  - tuple : ( plan, fraction )  
    - `plan` - RobotTrajectory : 動作計画軌道
    - `fraction` - float : ウェイポイントによって記述されたパスの割合 0.0〜1.0 / エラーの場合は -1.0

---
#### def execute(self, plan_msg, wait = True):

- 機能
  - 事前に動作計画された動作を実行
- 引数
  - `plan_msg` - Plan : 動作計画軌道
  - `wait` - bool : 動作の終了を待つか否か / デフォルト True
- 戻り値
  - MoveItErroeCode : MoveIt! エラーコード

---
#### def attach_object(self, object_name, link_name = "", touch_links = []):

- 機能
  - 動作計画シーン内にあるオブジェクトをロボットのリンクに接続
- 引数
  - `object_name` - str : ロボットに接続するオブジェクト名
  - `link_name` -str : オブジェクトを接続するロボットのリンク名 / デフォルト エンドエフェクタ
  - `touch_links` - [ str, str, ... ] : オブジェクトが干渉を考慮せずに接触することが許されるリンク群
- 戻り値
	- bool : Move Group ノードにオブジェクトの接続リクエストが送信されたときに True を返す
	/ これは接続リクエストの実行が成功したか否かは意味しない

---
#### def detach_object(self, name = ""):

- 機能
  - 指定したリンクからオブジェクトを切り離す
- 引数
  - `name` - str : オブジェクトを切り離すリンク名
    - リンク名が存在しない場合はオブジェクト名として解釈
    - 名前が指定されていない場合はグループ内のリンクに接続されている全てのオブジェクトを切り離そうとする
- 戻り値
  - bool : 切り離すオブジェクトが特定できない場合はエラーを生成

---
#### def pick(self, object_name, grasp = []):

- 機能
  - 名称のあるオブジェクトの持ち上げ
- 引数
  - `object_name` - str : 掴む対象オブジェクトの名称
  - `grasp` - Grasp or [ Grasp, Grasp, ... ] : 掴む動作の指定
- 戻り値
  - MoveItErroeCode : MoveIt! エラーコード

---
#### def place(self, object_name, location=None):

- 機能
  - オブジェクトを置く
- 引数
  - `object_name` - str : 置くオブジェクトの名称
  - `location` - PoseStamped, Pose, PlaceLocation : オブジェクトを置く場所
    / デフォルト None （ 環境内の安全な場所を検出してそこへ置く ）
- 戻り値
  - MoveItErroeCode : MoveIt! エラーコード

---
#### def set_support_surface_name(self, value):

- 機能
  - place() 動作用の支持サーフェス名を設定
- 引数
  - value - str : サーフェス名
- 戻り値
  - なし

---
#### def retime_trajectory(self, ref_state_in, traj_in, velocity_scaling_factor):

- 機能
  - 軌道のタイムスタンプを速度と加速度の制約を考慮して変更
- 引数
  - `ref_state_in` - RobotState : 現在のロボット全体の状態
  - `traj_in` - RobotTrajectory : タイムスタンプの再設定する動作計画軌道
  - `velocity_scaling_factor` - float : 速度係数
- 戻り値
  - RobotTrajectory : 動作計画軌道
---

---

### RobotCommander クラス

GitHub - [moveit_commander/src/moveit_commander/robot.py][ad535b3a]

GitHub - [moveit/moveit_ros/planning_interface/robot_interface/src/wrap_python_robot_interface.cpp][4cfc3b2d]

  [ad535b3a]: https://github.com/ros-planning/moveit_commander/blob/<$ROS_DISTRO>-devel/src/moveit_commander/robot.py "RobotCommander Class"

  [4cfc3b2d]: https://github.com/ros-planning/moveit/blob/<$ROS_DISTRO>-devel/moveit_ros/planning_interface/robot_interface/src/wrap_python_robot_interface.cpp "wrap_python_robot_interface"

---
#### def \_\_init\_\_(self, robot_description="robot_description"):

- 機能
  - コンストラクタ
- 引数
  - `robot_description` - str : ロボット名
- 戻り値
  - なし

---
#### def get_planning_frame(self):

- 機能
  - 動作計画で使用された参照フレーム名の取得
- 引数
  - なし
- 戻り値
  - str : リンク名


---
#### def get_root_link(self):

- 機能
  - ロボットモデルのルートリンク名の取得
- 引数
  - なし
- 戻り値
  - str : リンク名

---
#### def get_joint_names(self, group=None):

- 機能
  - 関節名の取得
- 引数
  - `group` - str : グループ名 / デフォルト None（ロボットの全関節名を返す）
- 戻り値
  - [ str, str, ... ] : 関節名のリスト

---
#### def get_link_names(self, group=None):

- 機能
  - リンク名の取得
- 引数
  - `group` - str : グループ名 / デフォルト None（ロボットの全リンク名を返す）
- 戻り値
  - [ str, str, ... ] : リンク名のリスト

---
#### def get_current_state(self):

- 機能
  - ロボットの現在の状態の RobotState 型メッセージの取得
- 引数
  - なし
- 戻り値
  - RobotStage : ロボットの現在の状態

---
#### def get_current_variable_values(self):

- 機能
  - ジョイント名とジョイント状態値のディクショナリとして現在の状態を取得
- 引数
  - なし
- 戻り値
  - dictionary : ジョイント名とジョイント状態値のディクショナリ

---
#### def get_joint(self, name):

- 機能
  - 関節の Joint クラスの取得
- 引数
  - `name` - str : 関節名
- 戻り値
  - Joint : Joint クラス

---
#### def get_link(self, name):

- 機能
  - リンクの Link クラスの取得
- 引数
  - `name` - str : 関節名
- 戻り値
  - Link : Link クラス

---
#### def get_group(self, name):

- 機能
  - グループ名を指定してその MoveGroupCommander クラスを取得
- 引数
  - `name` - str : グループ名
- 戻り値
  - MoveGroupCommander クラス :

---
#### def has_group(self, name):

- 機能
  - グループ名のグループが存在するかを確認
- 引数
  - `name` - str : グループ名
- 戻り値
  - bool : グループが存在するか否か

---
#### def get_default_owner_group(self, joint_name):

- 機能
  - 引数指定された関節名を含む最小のグループの名前の取得
- 引数
  - `joint_name` - str : 関節名
- 戻り値
  - str : グループ名

---

### Joint クラス（ RobotCommandeer 内クラス ）

---
#### def \_\_init\_\_(self, robot, name):

- 機能
  - コンストラクタ
- 引数
  - `robot` - RobotCommander : RobotCommander クラスインスタンス
  - `name` - str : 関節名
- 戻り値
  - なし

---
#### def name(self):

- 機能
  - 関節名の取得
- 引数
  - なし
- 戻り値
  - str : 関節名

---
#### def variable_count(self):

- 機能
  - 関節を記述する変数の数の取得
- 引数
  - なし
- 戻り値
  - int : 変数の数

---
#### def bounds(self):

- 機能
  - 関節リミットの最小値と最大値のリストもしくはそれらのセットリストの取得
- 引数
  - なし
- 戻り値
  - [ float, float ] or [ [float, float], ... ] : 関節リミットのリスト

---
#### def min_bound(self):

- 関節リミットの最小値もしくはそれらのセットリストの取得
- 引数
  - なし
- 戻り値
  - float or [ [float, float], ... ] : 関節リミットの最小値（のリスト）

---
#### def max_bound(self):

- 関節リミットの最大値もしくはそれらのセットリストの取得
- 引数
  - なし
- 戻り値
  - float or [ [float, float], ... ] : 関節リミットの最大値（のリスト）

---
#### def value(self):

- 機能
  - 関節の現在の値の取得
- 引数
  - なし
- 戻り値
  - float : 関節の現在の値

---
#### def move(self, position, wait=True):

- 機能
  - 関節を目標値に動かす
- 引数
  - `position` - float : 目標値
  - `wait` - bool : 動作完了を待つか否か / デフォルト True
- 戻り値
- MoveItErroeCode or bool : MoveIt! エラーコード もしくは False

---
#### def \_\_get_joint_limits(self):

- 機能
  - 関節リミットの最大値と最小値のリストのセットリストの取得
- 引数
  - なし
- 戻り値
  - [ float, float ] or [ [float, float], ... ] : 関節リミットのリスト

---

### Link クラス（ RobotCommandeer 内クラス ）

---
#### def \_\_init\_\_(self, robot, name):

- 機能
  - コンストラクタ
- 引数
  - `robot` - RobotCommander : RobotCommander クラスインスタンス
  - `name` - リンク名
- 戻り値
  - なし

---
#### def name(self):

- 機能
  - リンク名の取得
- 引数
  - なし
- 戻り値
  - str : リンク名

#### def pose(self):

- 機能
  - リンクの姿勢の取得
- 引数
  - なし
- 戻り値
  - PoseStamped : リンクの姿勢

---

---

### PlanningSceneInterface クラス

GitHub - [moveit_commander/src/moveit_commander/planning_scene_interface.py][844eea07]

GitHub - [moveit/moveit_ros/planning_interface/planning_scene_interface/src/wrap_python_planning_scene_interface.cpp][86935f4f]

  [844eea07]: https://github.com/ros-planning/moveit_commander/blob/<$ROS_DISTRO>-devel/src/moveit_commander/planning_scene_interface.py "GitHub - PlanningSceneInterface クラス"

  [86935f4f]: https://github.com/ros-planning/moveit/blob/<$ROS_DISTRO>-devel/moveit_ros/planning_interface/planning_scene_interface/src/wrap_python_planning_scene_interface.cpp "wrap_python_planning_scene_interface.cpp"

---
#### def \_\_init\_\_(self):

- 機能
  - コンストラクタ
- 引数
  - なし
- 戻り値
  - なし

---
#### def \_\_make_sphere(self, name, pose, radius):

- 機能
  - 球形状オブジェクトの作成
- 引数
  - `name` - str : オブジェクトの名前
  - `pose` - PoseStamped : オブジェクトの姿勢
  - `radius` - float : 球の半径 [m]
- 戻り値
  - ColllisionObject : オブジェクト

---
#### def add_sphere(self, name, pose, radius = 1):

- 機能
  - 球形状オブジェクトを作成して動作計画空間に設置
- 引数
  - `name` - str : オブジェクトの名前
  - `pose` - PoseStamped : オブジェクトの姿勢
  - `radius` - float : 球の半径 [m]
- 戻り値
  - なし

---
#### def \_\_make_box(self, name, pose, size):

- 機能
  - 箱形状オブジェクトの作成
- 引数
  - `name` - str : オブジェクトの名前
  - `pose` - PoseStamped : オブジェクトの姿勢
  - `size` - ( float, float, float ) : 三辺の各長さのタプル [m]
- 戻り値
  - ColllisionObject : オブジェクト

---
#### def add_box(self, name, pose, size = (1, 1, 1)):

- 機能
  - 箱形状オブジェクトを作成して動作計画空間に設置
- 引数
  - `name` - str : オブジェクトの名前
  - `pose` - PoseStamped : オブジェクトの姿勢
  - `size` - ( float, float, float ) : 三辺の各長さのタプル [m]
- 戻り値
  - なし

---
#### def \_\_make_mesh(self, name, pose, filename, scale = (1, 1, 1)):

- 機能
  - メッシュデータからオブジェクトを作成
- 引数
  - `name` - str : オブジェクトの名前
  - `pose` - PoseStamped : オブジェクトの姿勢
  - `filename` - str : メッシュデータファイル名
  - `scale` - ( float, float, float ) : スケール / デフォルト (1, 1, 1)
- 戻り値
  - ColllisionObject : オブジェクト

---
#### def add_mesh(self, name, pose, filename, size = (1, 1, 1)):

- 機能
  - メッシュデータからオブジェクトを作成して動作計画空間に設置
- 引数
  - `name` - str : オブジェクトの名前
  - `pose` - PoseStamped : オブジェクトの姿勢
  - `filename` - str : メッシュデータファイル名
  - `scale` - ( float, float, float ) : スケール / デフォルト (1, 1, 1)
- 戻り値
  - なし

---
#### def \_\_make_existing(self, name):

- 機能
  - オブジェクトが既に存在するときに使用される空のオブジェクトの作成
- 引数
  - `name` - str : オブジェクト
- 戻り値
  - ColllisionObject : オブジェクト

---
#### def add_plane(self, name, pose, normal = (0, 0, 1), offset = 0):

- 機能
  - 平面オブジェクトを作成して動作計画空間に設置
- 引数
  - `name` - str : オブジェクトの名前
  - `pose` - PoseStamped : オブジェクトの姿勢
  - `normal` - ( float, float, float ) : 面の法線ベクトル / デフォルト (0, 0, 1)
  - `offset` - float : オフセット量 [m] / デフォルト 0
- 戻り値
  - なし

---
#### def attach_mesh(self, link, name, pose = None, filename = '', size = (1, 1, 1), touch_links = []):

- 機能
  - メッシュデータをロボットリンクに接続
    - 指定があればメッシュオブジェクトを作成
- 引数
  - `link` - str : 接続するロボットリンク名
  - `name` - str : 接続するオブジェクトの名前
  - `pose` - PoseStamped : オブジェクトの姿勢
  - `filename` - str : メッシュデータファイル名
  - `size` - ( float, float, float ) : サイズスケール / デフォルト (1, 1, 1)
  - `touch_links` - - [ str, str, ... ] : オブジェクトが干渉を考慮せずに接触することが許されるリンク群
- 戻り値
  - なし

---
#### def attach_box(self, link, name, pose = None, size = (1, 1, 1), touch_links = []):

- 機能
  - 箱オブジェクトをロボットリンクに接続
    - 指定があれば箱オブジェクトを作成
- 引数
  - `link` - str : 接続するロボットリンク名
  - `name` - str : 接続するオブジェクトの名前
  - `pose` - PoseStamped : オブジェクトの姿勢
  - `size` - ( float, float, float ) : サイズ / デフォルト (1, 1, 1)
  - `touch_links` - - [ str, str, ... ] : オブジェクトが干渉を考慮せずに接触することが許されるリンク群
- 戻り値
  - なし

---
#### def remove_world_object(self, name = None):

- 機能
  - 動作計画空間からオブジェクトを削除
    - オブジェクト名の指定がなければ全オブジェクトを削除
- 引数
  - `name` - str : 削除するオブジェクト名
- 戻り値
  - なし

---
#### def remove_attached_object(self, link, name = None):

- 機能
  - リンクに接続している動作計画空間内のオブジェクトを削除
    - オブジェクト名の指定がなければリンクに接続している全オブジェクトを削除
- 引数
  - `link` - str : 削除するオブジェクトの接続しているリンク名
  - `name` - str : 削除するオブジェクト名
- 戻り値
  - なし

---
#### def get_known_object_names(self, with_type = False):

- 機能
  - 既知のオブジェクトの全名称の取得
- 引数
  - `with_type` - bool : 既知の型を持つオブジェクトのみを返すか否か / デフォルト False
- 戻り値
  - [ str, str, ... ] : オブジェクトの名称リスト

---
#### def get_known_object_names_in_roi(self, minx, miny, minz, maxx, maxy, maxz, with_type = False):

- 機能
  - 領域内にある既知のオブジェクトの全名称の取得
- 引数
  - `minx` - float : 領域のX方向最小値
  - `miny` - float : 領域のY方向最小値
  - `minz` - float : 領域のZ方向最小値
  - `maxx` - float : 領域のX方向最大値
  - `maxy` - float : 領域のY方向最大値
  - `maxz` - float : 領域のZ方向最大値
  - `with_type` - bool : 既知の型を持つオブジェクトのみを返すか否か / デフォルト False
- 戻り値
  - [ str, str, ... ] : オブジェクトの名称リスト

---
#### def get_objects(self, object_ids = []):

- 機能
  - リストで指定したオブジェクトデータの取得
- 引数
  - `object_ids` - [ str, str, ... ] : オブジェクト名のリスト（指定がなければ全て）
- 戻り値
  - { str:CollisionObject, str:CollisionObject, ... } : オブジェクトのディクショナリ

---
#### def get_attached_objects(self, object_ids = []):

- 機能
  - リンク接続しているリストで指定したオブジェクトデータの取得
- 引数
  - `object_ids` - [ str, str, ... ] : オブジェクト名のリスト（指定がなければ全て）
- 戻り値
  - { str:CollisionObject, str:CollisionObject, ... } : オブジェクトのディクショナリ

---

---

### conversions の関数

- GitHub - [moveit_commander/src/moveit_commander/conversions.py][0aa62cbe]

  [0aa62cbe]: https://github.com/ros-planning/moveit_commander/blob/<$ROS_DISTRO>-devel/src/moveit_commander/conversions.py "conversions functions"

---
#### def msg_to_string(msg):

- 機能
  - ROS メッセージから文字列への変換
- 引数
  - `msg` - ROS Message : ROS メッセージ
- 戻り値
  - str : 文字列

---
#### def msg_from_string(msg, data):

- 機能
  - 文字列からROSメッセージへの変換
- 引数
  - `msg` - ROS Message : ROS メッセージ
  - `data` - str : 文字列
- 戻り値
  - なし

---
#### def pose_to_list(pose_msg):

- 機能
  - Pose からリストへの変換
- 引数
  - `pose_msg` - Pose : Pose メッセージ
- 戻り値
  - [ float, float, ... ] : [ x, y, z, qx, qy, qz, qw ]

---
#### def list_to_pose(pose_list):

- 機能
  - リストから Pose への変換
- 引数
  - `pose_list` - [ float, float, ... ] : [ x, y, z, qx, qy, qz, qw ]
- 戻り値
  - Pose : Pose メッセージ

---
#### def list_to_pose_stamped(pose_list, target_frame):

- 機能
  - リストから PoseStamped への変換
- 引数
  - `pose_list` - [ float, float, ... ] : [ x, y, z, qx, qy, qz, qw ]
  - `target_frame` - str : 基準フレーム
- 戻り値
  - Pose : Pose メッセージ

---
#### def transform_to_list(trf_msg):

- 機能
  - Transform からリストへの変換
- 引数
  - `trf_msg` - Transform : Transform メッセージ
- 戻り値
  - [ float, float, ... ] : [ x, y, z, qx, qy, qz, qw ]

---
#### def list_to_transform(trf_list):

- 機能
  - リストから Transform への変換
- 引数
  - `trf_list` - [ float, float, ... ] : [ x, y, z, qx, qy, qz, qw ]
- 戻り値
  - Transform : Transform メッセージ

---

---

### roscpp_initializer の関数

- GitHub - [moveit_commander/src/moveit_commander/roscpp_initializer.py][f6bf1bd0]

  [f6bf1bd0]: https://github.com/ros-planning/moveit_commander/blob/<$ROS_DISTRO>-devel/src/moveit_commander/roscpp_initializer.py "roscpp_initializer functions"

---
#### def roscpp_initialize(args):

- 機能
  - moveit_commander プロセスの初期化
- 引数
  - `args` - [ str, str, ... ] : スクリプトの名前とコマンドライン引数のリスト
- 戻り値
  - なし

---
#### def roscpp_shutdown():

- 機能
  - moveit_commander プロセスの終了
- 引数
  - なし
- 戻り値
  - なし

---

-----

## チュートリアルパッケージ

### moveit_tutorial_tools の関数

---
#### def init_node( node_name = "commander_example" ):

- 機能
  - ROS と Qtアプリケーション の初期化
  - ロボットの動作グループ（ Move Group ）の表示
- 引数
  - `node_name` : str - 初期化する ROS ノードにつける名称 / デフォルト "commander_example"

---
#### def question_yn( qmsg='Message', title='Question' ):

- 機能
  - Yes / No を問い合わせて Yes が選択された場合にのみ `True` を返す
  - PyQt の QMessageBox() を利用
- 引数
  - `qmsg` - str : 問い合わせメッセージの文字列 / デフォルト 'Message'
  - `title` - str : メッセージボックスウィンドウのタイトル文字列 / デフォルト 'Question'
- 戻り値
  - bool - Yes が選択された場合のみ `True` を返す / その他は `False`

---
#### def get_current_target_pose( target_frame_id, base_frame_id, timeout = 1.0 ):

- 機能
  - 引数で指定されたのフレーム間の TF を返す
- 引数
  - `target_frame_id` - str : TF の値を得たいフレームの名称
	- `base_frame_id` - str : TF 変換の基準とするフレームの名称
	- `timeout` - float : TF 変換のタイムアウト時間 [sec] / デフォルト 1.0

---
#### def make_waypoints_example( rtype="NEXTAGE" ):

- 機能
  - 動作例用の複数の姿勢の作成
- 引数
  - `rtype` - str 型 : ロボットのタイプ名 / デフォルト "NEXTAGE"
- 戻り値
  - [ Pose, Pose, ... ] : 動作例用の複数のポーズのリスト

---
#### def make_waypoints_rectangular( dp_a=[0.25, 0.0, 0.1], dp_b=[0.45, -0.2, 0.1], rpy=[0.0,0.0,0.0] ):

- 機能
  - 四角形を描く動作の各頂点の姿勢の作成
- 引数
  - `dp_a` - [float, float, float] : 1つ目の四角形の対角点 / デフォルト [0.25, 0.0, 0.1]
  - `dp_b` - [float, float, float] : 2つ目の四角形の対角点 / デフォルト [0.45, -0.2, 0.1]
  - `rpy` - [float, float, float] : 四角形を描く動作時の姿勢 Roll,Pitch,Yaw [rad] / デフォルト [0.0,0.0,0.0]
- 戻り値
  - [ Pose, Pose, ... ] : 四角形を描く動作の各頂点の姿勢

---
#### def make_waypoints_circular( center=[0.3, -0.2, 0.1], radius=0.1 ,steps=12, rpy=[0.0,0.0,0.0] ):

- 機能
  - 円を描く動作の円周上の姿勢の作成
- 引数
  - `center` - [float, float, float] : 円の中心の位置 / デフォルト [0.3, -0.2, 0.1]
  - `radius` - float : 円の半径 [m] / デフォルト 0.1
	- `steps` - int : 円周の分割数 / デフォルト 12
  - `rpy` - [float, float, float] : 円を描く動作時の姿勢 Roll,Pitch,Yaw [rad] / デフォルト [0.0,0.0,0.0]
- 戻り値
  - [ Pose, Pose, ... ] : 円を描く動作の各頂点の姿勢

---


<!-- EOF -->
