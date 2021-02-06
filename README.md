# Scikit-surgerycalibration-ros

This package provides interfaces between ROS2 and scikit-surgerycalibration.

## Interfaces
  - Pivot calibration   
     This uses TF to acquire frames and perform a pivot calibration finally publishing the calibration in TF.
     
     
 # Docker
 There is a docker file to install this in a container.
 ## Build
```commandline
docker build --pull --rm -f ./Dockerfile  -t scikit-surgerycalibration-ros:latest .
```

## RUN

For development
```commandline
docker run -it \
--user=$(id -u $USER):$(id -g $USER) \
--group-add sudo \
--env="DISPLAY" \
--env=QT_X11_NO_MITSHM=1 \
--workdir="/dev_ws/src" \
--volume="/home/$USER:/home/$USER" \
--volume="/etc/group:/etc/group:ro" \
--volume="/etc/passwd:/etc/passwd:ro" \
--volume="/etc/shadow:/etc/shadow:ro" \
--volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
scikit-surgerycalibration-ros:latest
```

TODO: when tools are implemented, be able to run with this. Will need to add checks in the nodes for configuration.
For running the tools
```commandline
docker run -it \
--rm \
--net=host \
scikit-surgerycalibration-ros:latest <tool_to_run>
```
