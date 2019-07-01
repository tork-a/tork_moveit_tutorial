^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tork_moveit_tutorial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* add KHI duaro tutorial (ROS Kinetic only) (`#34 <https://github.com/tork-a/tork_moveit_tutorial/issues/34>`_)
* Add kinetic images (`#30 <https://github.com/tork-a/tork_moveit_tutorial/issues/30>`_)

  * remove html comment lines
  * add ros kinetic screen capture images

* Contributors: Yosuke Yamamoto

0.0.6 (2019-03-13)
------------------
* Enable to run both indigo and kinetic (`#27 <https://github.com/tork-a/tork_moveit_tutorial/issues/27>`_)

  * mod for PyQt4/PyQt5 compatible codes
  * mod for ROS Kinetic (PyQt4->PyQt5) and gazobo_ros dependency

* Fix `#13 <https://github.com/tork-a/tork_moveit_tutorial/issues/13>`_ , `#14 <https://github.com/tork-a/tork_moveit_tutorial/issues/14>`_ , `#15 <https://github.com/tork-a/tork_moveit_tutorial/issues/15>`_ , `#16 <https://github.com/tork-a/tork_moveit_tutorial/issues/16>`_ , (`#17 <https://github.com/tork-a/tork_moveit_tutorial/issues/17>`_)

  * Fix wrong link strings
  * Correct minor typos
  * Add section for license
  * Add link to origian GitHub page
  * Fix if/then sentences
  * mod for kinetic tutorial, correct typos, change '<br>' to 2 spaces (markdown new line)

* remove footer in CC section, close `#14 <https://github.com/tork-a/tork_moveit_tutorial/issues/14>`_ (`#22 <https://github.com/tork-a/tork_moveit_tutorial/issues/22>`_)
* enable to compile both indigo and kinetic document (`#25 <https://github.com/tork-a/tork_moveit_tutorial/issues/25>`_)

  * migrate to circleci2
  * enable to compile both indigo and kinetic
  * replace indigo to <\$ROS_DISTRO>  usg gpp to preprocess ROS_DISTRO, see  http://files.nothingisreal.com/software/gpp/gpp.html, https://randomdeterminism.wordpress.com/2012/06/01/how-i-stopped-worring-and-started-using-markdown-like-tex/

* Update moveit-tutorial_ja_robot-simulator.md (`#21 <https://github.com/tork-a/tork_moveit_tutorial/issues/21>`_)
* Fix dpkg error in CircleCI (`#19 <https://github.com/tork-a/tork_moveit_tutorial/issues/19>`_)

  * Specify the version of sphinx and recommonmark
  * Fix dpkg error in CircleCI

* [tork_moveit_tutorial/script] fix import module name (`#10 <https://github.com/tork-a/tork_moveit_tutorial/issues/10>`_)

  * [tork_moveit_tutorial/doc] add missing bar in function reference.
  * [tork_moveit_tutorial/script/nextage_moveit_tutorial_poses_botharms.py] fix arguments for pose target 1.
  * [tork_moveit_tutorial/script] fix import package name: moveit_tutorial_tools -> tork_moveit_tutorial.

* Contributors: Kei Okada, Masaki Murooka, Ryosuke Tajima, Yosuke Yamamoto

0.0.5 (2018-02-06)
------------------
* run debbuild within circleci (`#9 <https://github.com/tork-a/tork_moveit_tutorial/issues/9>`_)
  * do not copy but rename pdf file
  * enable docker
  * run debbuild within circleci
* correct image of opening moveit_armarker.rviz config file (the file directory was wrong) (`#8 <https://github.com/tork-a/tork_moveit_tutorial/issues/8>`_)
* Contributors: Tokyo Opensource Robotics Developer 534, Yosuke Yamamoto

0.0.4 (2018-01-15)
------------------
* Apply feedback (`#6 <https://github.com/tork-a/tork_moveit_tutorial/issues/6>`_)
  * add images/gazebo_startup_error.png
  * add link to gazebo-start-not-show-robot
  * add comment on clean exit
  * add comment on ABORTED: Solution found but controller failed during execution
  * add comment about: TrajectoryExecution will use old action capability.
  * add link to official baxter setup
  * explain first gazebo startup needs wait
* Contributors: Tokyo Opensource Robotics Developer 534

0.0.3 (2017-12-19)
------------------
* Fix python src path (`#5 <https://github.com/tork-a/tork_moveit_tutorial/issues/5>`_)
  * disable virtualenv
  * forget to add setup.py
  * change moveit-tutorial_ja_robot-python_basic.md due to moving demo program in moveit_tutorial_tools.py into script/demo.py
  * move demo program in moveit_tutorial_tools.py into script/demo.py
  * fix doc due to python file format
  * mv script/moveit_tutorial_tools.py to src/tork_moveit_tutorial/
  * clean up files, and add install rule
  * add more depends
* Contributors: Tokyo Opensource Robotics Developer 534

0.0.2 (2017-12-15)
------------------
* clean up sections, add cc (`#4 <https://github.com/tork-a/tork_moveit_tutorial/issues/4>`_)
* Contributors: Tokyo Opensource Robotics Developer 534

0.0.1 (2017-12-14)
------------------
* Add circle.yml
* Initial Commit
* Contributors: Tokyo Opensource Robotics Developer 534, Yosuke Yamamoto
