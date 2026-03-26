#! /bin/bash
pwd 
cd ..
if [ -e pcl-1.14.0 ]
then
cd pcl-1.14.0
pwd
mkdir build
cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
else
git clone https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.14.0.zip
if [ -e pcl-1.14.0 ]
then
cd pcl-1.14.0
pwd
mkdir build
cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
fi
echo "如果无法安装请访问:https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.14.0.zip"

fi