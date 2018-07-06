echo "Building ROS nodes"

cd ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
