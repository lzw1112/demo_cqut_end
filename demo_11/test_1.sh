#! /bin/bash
sudo apt update
sudo apt install cmake
sudo apt-get install ros-humble-gazebo-dev
sudo apt-get install ros-humble-gazebo-msgs
sudo apt-get install  ros-humble-gazebo-ros2-control
sudo apt-get install  ros-humble-gazebo-plugins
sudo apt-get install  ros-humble-gazebo-plugins-dbgsym
cd ..
pwd
sudo apt-get update
echo "开始安装ceres"
if [ -e ceres-solver ]
then
cd ceres-solver
pwd
mkdir build
cd build
cmake ..
make -j4
sudo make install
else
git clone git clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver
if [ -e ceres-solver ]
then
cd ceres-solver
pwd
mkdir build
cd build
cmake ..
make -j4
sudo make install
fi
echo "如果无法安装请访问: https://ceres-solver.googlesource.com/ceres-solver ceres-solver,然后把下载的文件改名为ceres-solver，重新使用脚本"

fi
