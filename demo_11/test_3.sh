#! /bin/bash
pwd
cd ..
sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev libblas-dev 

if [ -e ThirdParty-ASL ]
    then
    cd ThirdParty-ASL
    pwd
    sudo ./get.ASL
    sudo ./configure
    sudo make
    sudo make install
    cd ..
else
    git clone https://github.com/coin-or-tools/ThirdParty-ASL.git
if [ -e ThirdParty-ASL ]
    then
    cd ThirdParty-ASL
    pwd
    sudo ./get.ASL
    sudo ./configure
    sudo make
    sudo make install
    pwd
    cd ..
    echo "如果无法安装请访问:https://github.com/coin-or-tools/ThirdParty-ASL.git,解压下载下来的fmt-main,重命名为fmt.然后重新执行这个脚本"
fi
fi
if [ -e ThirdParty-HSL ]
    then
    #git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
    cd ThirdParty-HSL
    git clone https://github.com/CHH3213/testCPP/blob/master/coinhsl.zip
    sudo ./configure
    sudo make
    sudo make install
    cd ..
else 
git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
if [ -e ThirdParty-HSL ]
    then
    cd ThirdParty-HSL
    git clone https://github.com/CHH3213/testCPP/blob/master/coinhsl.zip
    pwd
    sudo ./configure
    sudo make
    sudo make install
    cd ..
    echo 'ending'
fi
fi
if [ -e ThirdParty-Mumps ]
then
#git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
cd ThirdParty-Mumps
sudo ./get.Mumps
sudo ./configure
sudo make
sudo make install
cd ..
else 
    git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
    if [ -e ThirdParty-Mumps ]
        then
        cd ThirdParty-Mumps
        sudo ./get.Mumps
        sudo ./configure
        sudo make
        sudo make install
        cd ..
        echo 'ending'
    fi
fi
if [ -e Ipopt ]
then
#git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
cd Ipopt
mkdir build
cd build
sudo ../configure
sudo make
sudo make test
sudo make install
cd ..
cd /usr/local/include
sudo cp coin-or coin -r
sudo ln -s /usr/local/lib/libcoinmumps.so.3 /usr/lib/libcoinmumps.so.3
sudo ln -s /usr/local/lib/libcoinhsl.so.2 /usr/lib/libcoinhsl.so.2
sudo ln -s /usr/local/lib/libipopt.so.3 /usr/lib/libipopt.so.3
else 
    git clone https://github.com/coin-or/Ipopt.git
    if [ -e Ipopt ]
        then
        cd Ipopt
        mkdir build
        cd build
        sudo ../configure
        sudo make
        sudo make test
        sudo make install
        cd ..
        cd /usr/local/include
        sudo cp coin-or coin -r
        sudo ln -s /usr/local/lib/libcoinmumps.so.3 /usr/lib/libcoinmumps.so.3
        sudo ln -s /usr/local/lib/libcoinhsl.so.2 /usr/lib/libcoinhsl.so.2
        sudo ln -s /usr/local/lib/libipopt.so.3 /usr/lib/libipopt.so.3
    fi
fi
