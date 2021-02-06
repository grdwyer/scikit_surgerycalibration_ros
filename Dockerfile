FROM ros:foxy

# Install needed ros components and pip3
RUN apt-get update && apt-get install -y \
    ros-foxy-rqt \
    python3-pip \
    python3-pykdl \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install git+https://github.com/UCL/scikit-surgerycalibration

RUN mkdir -p /dev_ws/src/scikit-surgerycalibration
COPY . /dev_ws/src/scikit-surgerycalibration-ros/

# Build the workspace so you're ready to go
WORKDIR /dev_ws
RUN ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash &&\
    rosdep install -i --from-path src --rosdistro foxy -y &&\
    colcon build"]

CMD ["bash"]
