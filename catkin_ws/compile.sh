#!/bin/bash
catkin_make --only-pkg-with-deps $(ls src/ | grep -v tobi_robot | grep -v katana_driver | grep -v meka-ros-pkg)
