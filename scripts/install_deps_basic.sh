#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

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

echo "如果要编译bodyreader,需要下载ASTRA SDK,并且安装,然后将2个lib添加至bashrc变量,具体见/bodyreader/CMakeLists.txt"
cd thirdparties/AstraSDK/
bash install/install.sh
check_success
echo "export ASTRA_SDK_INCLUDE=$SDK_PATH/include" >> ~/.bashrc
echo "export ASTRA_SDK_LIB=$SDK_PATH/lib" >> ~/.bashrc
check_success
source ~/.bashrc
check_success
cd ../../

catkin_make --pkg lslidar_msgs lslidar_driver lslidar # 先编译lslidar的包
check_success





# catkin_make
# check_success