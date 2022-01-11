^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tork_moveit_tutorial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2022-01-11)
------------------
* more fix for melodic .circleci (`#57 <https://github.com/tork-a/tork_moveit_tutorial/issues/57>`_)
* Contributors: Tokyo Opensource Robotics Developer 534

0.1.0 (2022-01-11)
------------------
* add melodic sections (`#55 <https://github.com/tork-a/tork_moveit_tutorial/issues/55>`_)
* add example with myCobot (`#54 <https://github.com/tork-a/tork_moveit_tutorial/issues/54>`_)
* fix typo MINAS-TRA1 -> KHI Duaro (`#53 <https://github.com/tork-a/tork_moveit_tutorial/issues/53>`_)
* mod for kinetic with checking ROS version and normalizing orientation (`#47 <https://github.com/tork-a/tork_moveit_tutorial/issues/47>`_)
* fix CI (`#49 <https://github.com/tork-a/tork_moveit_tutorial/issues/49>`_)
* fix a typo 'movieit' to 'moveit' (`#44 <https://github.com/tork-a/tork_moveit_tutorial/issues/44>`_)
* Bump jinja2 from 2.11.1 to 2.11.3 in /.circleci (`#52 <https://github.com/tork-a/tork_moveit_tutorial/issues/52>`_)
* Bump urllib3 from 1.25.8 to 1.26.5 in /.circleci (`#50 <https://github.com/tork-a/tork_moveit_tutorial/issues/50>`_)
* Bump babel from 2.8.0 to 2.9.1 in /.circleci (`#56 <https://github.com/tork-a/tork_moveit_tutorial/issues/56>`_)

* Contributors: Kei Okada, Tokyo Opensource Robotics Developer 534, Yosuke Yamamoto, dependabot[bot]

0.0.10 (2020-01-23)
-------------------
* mod a file name of the additional chapter (`#43 <https://github.com/tork-a/tork_moveit_tutorial/issues/43>`_)

  * fix long line to fit page size
  * fix to pass 'prefix' error
  * mod markdown of the additional chapter
  * remove ifeq-endif in the additional chapter
  * mod a file name of the additional chapter

* Contributors: Tokyo Opensource Robotics Developer 534, Yosuke Yamamoto

0.0.9 (2020-01-22)
------------------
* fix documents within 110 pages (`#42 <https://github.com/tork-a/tork_moveit_tutorial/issues/42>`_)
* add a chapter of running your own program for ROS kinetic (`#41 <https://github.com/tork-a/tork_moveit_tutorial/issues/41>`_)
* Contributors: Tokyo Opensource Robotics Developer 534, Yosuke Yamamoto

0.0.8 (2020-01-22)
------------------
* Move kinetic baxter (`#36 <https://github.com/tork-a/tork_moveit_tutorial/issues/36>`_)

  * move compute_cartesian_path() descriptions before the waypoints plan descriptions
  * mod docs for ROS kinetic with KHI duaro robot instead of Baxter Research Robot
  * add info about quaternion
  * add notes of EEF and Quarternion
  * mod. a wrong filename
  * wrote running-your-own-program to the end
  * remove baxter robot in kinetic tutorial

* Contributors: Yosuke Yamamoto

0.0.7 (2019-07-01)
------------------
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
