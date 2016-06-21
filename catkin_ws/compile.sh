#!/bin/bash
catkin_make --pkg -pkg $(ls src/ | grep -v tobi_robot | grep -v katana_driver | grep -v meka-ros-pkg)
