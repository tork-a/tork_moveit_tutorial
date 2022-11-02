
# はじめに

本チュートリアルは
GUI（グラフィカル・ユーザ・インタフェース）から
ロボットやロボットシミュレーションを動かしたことはあるものの
それをプログラムから動かすことは行ったことがない人が
プログラミングを始められる，
そして発展的に様々なロボット動作プログラミングができるようになること
を目標としています．

本チュートリアルの PDF 形式のファイル，および Markdown 形式のソースコードは
以下の場所で公開されています．
コマンド等の文字列は，ここからコピー＆ペーストすると便利です．

- tork-a/tork_moveit_tutorial : [https://github.com/tork-a/tork_moveit_tutorial](https://github.com/tork-a/tork_moveit_tutorial)


## ROS と MoveIt!

ロボットソフトウェア開発で近年広く使われるようになっているのが
**ROS (ロス / Robot Operating System)** です．

- ROS : [http://wiki.ros.org/ja](http://wiki.ros.org/ja)
> ROS (Robot Operating System)はソフトウェア開発者のロボット・アプリケーション作成を支援するライブラリとツールを提供しています. 具体的には, ハードウェア抽象化, デバイスドライバ，ライブラリ，視覚化ツール, メッセージ通信，パッケージ管理などが提供されています. ROSはオープンソースの一つ, BSDライセンスにより, ライセンス化されています.

ROS でのロボット動作計画で頻繁に使用されるのが **MoveIt!** の動作計画機能です．
MoveIt! は GUI での操作を提供していますが，それだけではなく
そのプログラミングインタフェースである **MoveIt! Commander** が提供されていますので，
プログラミング言語から MoveIt! の機能を利用してロボットを動かすこともできます．

MoveIt! Commander のプログラミングインタフェースには Python や C++ があります．

- MoveIt! : [http://moveit.ros.org/](http://moveit.ros.org/)
- MoveIt! Tutorials : [http://docs.ros.org/<$ROS_DISTRO>/api/moveit_tutorials/html/](http://docs.ros.org/<$ROS_DISTRO>/api/moveit_tutorials/html/)


## チュートリアルの構成

本チュートリアルでは MoveIt! Commander を使って
ロボットの動作プログラミングを行います．

チュートリアルの構成は次のようになっています．

- [はじめに（本章）](moveit-tutorial_ja_introduction.md)
- [ロボットシミュレータを使う](moveit-tutorial_ja_robot-simulator.md)
- [プログラムでロボットを動かす](moveit-tutorial_ja_robot-python_basic.md)
- [発展的なロボットプログラミング](moveit-tutorial_ja_robot-python_advanced.md)
<$ifeq <$ROS_DISTRO>|kinetic>
- [独自プログラムの実行](moveit-tutorial_ja_running-your-own-program.md)
<$endif>
- [トラブルシューティング](moveit-tutorial_ja_trouble-shooting.md)
- [クラス・関数リファレンス](moveit-tutorial_ja_reference-class-functions.md)
- [Python チュートリアル](moveit-tutorial_ja_python.md)
<$if <$ROS_DISTRO>==melodic|<$ROS_DISTRO>==noetic>
- [実機の使用法](moveit-tutorial_ja_how-to-use-real-mycobot.md)
<$endif>

本チュートリアルのロボットプログラミングではプログラム言語 Python を使いますが，
Python の知識がなくてもコマンドやプログラムを実行できるように構成されています．

また，ロボット動作ではシミュレーションでのプログラミング実行が記述されていますので，
実機のロボットを扱える環境にない人や
ロボットプログラミングに慣れていないので先ずはシミュレーションからという人でも
プログラミングを行えます．


<!-- EOF -->
