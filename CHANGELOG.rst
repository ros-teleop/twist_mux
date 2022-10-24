^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package twist_mux
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.3 (2022-10-24)
------------------
* Fix tests for Python3 (`#40 <https://github.com/ros-teleop/twist_mux/issues/40>`_)
* [ROS1] Fix Noetic release / Python3 (`#39 <https://github.com/ros-teleop/twist_mux/issues/39>`_)
  Resolves `#22 <https://github.com/ros-teleop/twist_mux/issues/22>`_
* [ROS1] Add GitHub Actions CI (`#38 <https://github.com/ros-teleop/twist_mux/issues/38>`_)
* Contributors: Wolfgang Merkt

3.1.2 (2022-08-18)
------------------
* Install joystick_relay.py with catkin_install_python (`#37 <https://github.com/ros-teleop/twist_mux/issues/37>`_)
* Some more logical operator fixes
* Use standard logical or operator
* Contributors: Bence Magyar, Tobias Fischer, Wolfgang Merkt

3.1.1 (2020-06-02)
------------------
* Bump CMake version to avoid CMP0048
* Contributors: Bence Magyar

3.1.0 (2018-06-25)
------------------

3.0.0 (2016-07-12)
------------------
* Merge pull request `#8 <https://github.com/ros-teleop/twist_mux/issues/8>`_ from ros-teleop/update_jade_from_indigo_devel
  fix queue_size SyntaxWarning
* fix queue_size SyntaxWarning
* Update README.md
  Add documentation link and build status.
* Contributors: Enrique Fernández Perdomo, Jeremie Deray

2.0.0 (2015-05-22)
------------------
* Add rosindex tags
  http://rosindex.github.io/contribute/metadata/#metadata-elements
* Add missed test dependencies on rospy, rostopic
* Add .gitignore for *.pyc in test
* Fix rostest dependency
* Contributors: Enrique Fernandez

1.0.3 (2015-04-04)
------------------
* Update maintainer email
* Contributors: Enrique Fernandez

1.0.2 (2014-11-14)
------------------
* Increase version to 1.0.1 for indigo release
* Add diagnostic_updater dependency
* Contributors: Enrique Fernández

0.0.1 (2014-11-13)
------------------
* Add twist multiplexer
* Contributors: Enrique Fernandez, Siegfried Gevatter, Paul Mathieu
