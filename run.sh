#!/usr/bin/env bash
#5.5.0/run.sh

port="$1"

echo -e "# Crazyradio (normal operation) \nSUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1915\", ATTRS{idProduct}==\"7777\", MODE=\"0664\", GROUP=\"plugdev\" \n# Bootloader \nSUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1915\", ATTRS{idProduct}==\"0101\", MODE=\"0664\", GROUP=\"plugdev\"" | sudo tee /etc/udev/rules.d/99-crazyradio.rules
echo -e "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", MODE=\"0664\", GROUP=\"plugdev\"" | sudo tee /etc/udev/rules.d/99-crazyflie.rules

source /opt/ros/noetic/setup.bash  && cd crazyswarm/ros_ws/src/crazyswarm/scripts/pycrazyswarm/cfsim && make
source /opt/ros/noetic/setup.bash && cd crazyswarm/ros_ws && catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo

cd crazyfly/isse_ws && catkin_make -DCATKIN_BLACKLIST_PACKAGES="rviz_isse_panels"
cd ../..
echo "export PYTHONPATH=\${PYTHONPATH}:crazyswarm/ros_ws/src/crazyswarm/scripts" >> crazyfly/isse_ws/devel/setup.bash
echo "export PYTHONPATH=\${PYTHONPATH}:crazyswarm/ros_ws/src/crazyswarm/scripts" >> crazyfly/isse_ws/devel/setup.sh
echo "export PYTHONPATH=\${PYTHONPATH}:crazyswarm/ros_ws/src/crazyswarm/scripts" >> crazyfly/isse_ws/devel/setup.zsh
cp AbstractVirtualCapability.py ros_ws/src/isse_crazy/
cp IsseCrazyCopter.py ros_ws/src/isse_crazy/

source crazyfly/isse_ws/devel/setup.bash && cd /ros_ws && catkin_make
cd ../

source /crazyswarm/ros_ws/devel/setup.bash && source /ros_ws/devel/setup.bash && roslaunch isse_crazy crazyfly.launch semantix_port:="$port"