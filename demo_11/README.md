！！！！方案1：
    依次执行下面命令：
                    chmod +x test.sh
                    sudo ./test.sh
                    chmod +x test_1.sh
                    sudo ./test_1.sh
                    chmod +x test_2.sh
                    sudo ./test_2.sh
                    chmod +x test_3.sh
                    sudo ./test_3.sh
                    这里如果test_3.sh无法全部执行，需要手动下载https://github.com/CHH3213/testCPP/blob/master/coinhsl.zip
                    然后把这个文件解压到ThirdParty-HSL这个文件夹里面，重新执行sudo ./test_3.sh
                    chmod +x test_4.sh
                    sudo ./test_4.sh
                    如果无法下载，请手动下载https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.14.0.zip
                    然后解压放到主目录，再次执行sudo ./test_4.sh



！！！！方案2：
    1.安装cmake,至少高于3.7
    2 安装gazebo-dev,ros-humble-gazebo-msgs,ros-humble-gazebo-plugins,ros-humble-gazebo-plugins-dbgsym
    3 安装Sophus
    4 安装ceresgit clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver
        
        cd ceres-solver
        git reset --hard 6a74af202d83cf31811ea17dc66c74d03b89d79e
        mkdir build
        cd build
    5 安装https://github.com/fmtlib/fmt.gitsource 
    6 gazebo打不开 source /usr/share/gazebo/setup.bash
    7 启动小车ros2 launch cqut acceleeration.launch.py
    然后在当前目录启动rqt,其的话题cqut_res_state,勾选第三个选项，启动即可
    8 安装ipopt和cppid:
        安装依赖sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev libblas-dev 
        安装ASL：
        git clone https://github.com/coin-or-tools/ThirdParty-ASL.git
        cd ThirdParty-ASL
        sudo ./get.ASL
        sudo ./configure
        sudo make
        sudo make install
        cd ..8
        安装HSL:
        git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
        cd ThirdParty-HSL
        # 接下来需要下载coinhsl文件，并解压到ThirdParty-HSL目录下:https://github.com/CHH3213/testCPP/blob/master/coinhsl.zip
        在ThirdParty-HSL目录下，执行以下命令
        sudo ./configure
        sudo make
        sudo make install
        cd ..
        安装MUMPS：
        git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
        cd ThirdParty-Mumps
        sudo ./get.Mumps
        sudo ./configure
        sudo make
        sudo make install
        cd ..
        安装Ipopt:
        git clone https://github.com/coin-or/Ipopt.git
        cd Ipopt
        mkdir build
        cd build
        sudo ../configure
        sudo make
        sudo make test
        sudo make install
        改善环境：
        cd /usr/local/include
        sudo cp coin-or coin -r
        sudo ln -s /usr/local/lib/libcoinmumps.so.3 /usr/lib/libcoinmumps.so.3
        sudo ln -s /usr/local/lib/libcoinhsl.so.2 /usr/lib/libcoinhsl.so.2
        sudo ln -s /usr/local/lib/libipopt.so.3 /usr/lib/libipopt.so.3
    9 更改路径/home/lzw/demo_11/src/simulation/cqut_algorithm/cqut_planning/skidpad_detect/config/skidpad_detector.yaml里面




