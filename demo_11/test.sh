#! /bin/bash
pwd
cd ..

sudo apt-get update
sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
echo "开始安装Sophus"
if [ -e Sophus ]
then
cd Sophus
pwd
mkdir build
cd build
cmake ..
make -j4
sudo make install
else
git clone https://github.com/strasdat/Sophus.git
if [ -e Sophus ]
then
cd Sophus
pwd
mkdir build
cd build
cmake ..
make -j4
sudo make install
fi
echo "如果无法安装请访问:https://github.com/strasdat/Sophus.git,解压下载下来的Sophus-main,重命名为Sophus.然后重新执行这个脚本"

fi