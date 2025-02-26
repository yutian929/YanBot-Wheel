#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

sudo apt install python3-pip
check_success

sudo apt-get install libuvc-dev
check_success

sudo apt-get install libgeographic-dev
check_success

rosdep install --from-paths src --ignore-src -r -y
check_success

sudo apt install -y libeigen3-dev
check_success

# Check if /usr/include/Eigen already exists
if [ ! -d "/usr/include/Eigen" ]; then
    echo "Copying Eigen headers to /usr/include/Eigen"
    sudo cp -rf /usr/include/eigen3/Eigen /usr/include/Eigen -R
    check_success
else
    echo "/usr/include/Eigen already exists, skipping copy"
fi

sudo apt-get install libpcap-dev
check_success

sudo apt-get install python3-testresources
check_success

sudo pip3 install --upgrade rosdep
check_success

sudo apt install libcjson1 libcjson-dev
check_success

sudo apt install ros-noetic-robot-pose-ekf
check_success

sudo apt install ros-noetic-rtabmap-ros
check_success

echo "如果要编译bodyreader,需要下载ASTRA SDK,并且安装,然后将2个lib添加至bashrc变量,具体见/bodyreader/CMakeLists.txt"
cd src/thirdparties/AstraSDK/
bash install/install.sh
check_success
SDK_PATH=`pwd`
echo "export ASTRA_SDK_INCLUDE=$SDK_PATH/include" >> ~/.bashrc
echo "export ASTRA_SDK_LIB=$SDK_PATH/lib" >> ~/.bashrc
check_success
source ~/.bashrc
check_success
cd ../../../

echo "还需要移植.rules文件到/etc/udev/rules.d/目录下"
