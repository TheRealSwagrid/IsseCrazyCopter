FROM ros:noetic
SHELL ["/bin/bash", "-c"]

ENV CSW_PYTHON=python3
ENV DEBIAN_FRONTEND=noninteractive
ENV semantix_port=7500
ENV xmlrpc_port=45100
ENV tcpros_port=45101
ENV ROS_IP=127.0.0.1
ENV ROS_MASTER_URI=http://127.0.0.1:11311

# ROS-Noetic Setup
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update && apt-get install -y curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN sudo apt-get update

RUN apt-get update && apt-get install -y python-is-python3 python3-pip git iputils-ping
RUN apt-get update && apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential ros-noetic-vrpn-client-ros swig lib${CSW_PYTHON}-dev ${CSW_PYTHON}-pip
RUN apt-get update && apt-get install -y ros-noetic-tf ros-noetic-tf-conversions libpcl-dev libusb-1.0-0-dev
RUN ${CSW_PYTHON} -m pip install pytest numpy=="1.24.0" PyYAML scipy
RUN rosdep update

# Build CrazySwarm
RUN source /opt/ros/noetic/setup.bash && git clone https://github.com/USC-ACTLab/crazyswarm.git && cd crazyswarm && ./build.sh


#RUN bash crazyswarm/pc_permissions.sh
#RUN usermod -a -G plugdev $USER
RUN echo -e "# Crazyradio (normal operation) \nSUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1915\", ATTRS{idProduct}==\"7777\", MODE=\"0664\", GROUP=\"plugdev\" \n# Bootloader \nSUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1915\", ATTRS{idProduct}==\"0101\", MODE=\"0664\", GROUP=\"plugdev\"" | sudo tee /etc/udev/rules.d/99-crazyradio.rules
RUN echo -e "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", MODE=\"0664\", GROUP=\"plugdev\"" | sudo tee /etc/udev/rules.d/99-crazyflie.rules
#RUN udevadm control --reload-rules && udevadm trigger

# Build CrazyFly 
COPY /ros_ws /ros_ws
COPY AbstractVirtualCapability.py ros_ws/src/isse_crazy
COPY CrazyFly.py ros_ws/src/isse_crazy

RUN source crazyswarm/ros_ws/devel/setup.bash && cd /ros_ws && catkin_make

ENTRYPOINT ["/ros_entrypoint.sh"]
ENV PYTHONPATH=$PYTHONPATH:/crazyswarm/ros_ws/src/crazyswarm/scripts

CMD source /crazyswarm/ros_ws/devel/setup.bash && source /ros_ws/devel/setup.bash && roslaunch isse_crazy crazyfly.launch semantix_port:=${semantix_port} cf_id:=8
