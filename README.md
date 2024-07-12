# vk_sdk ros2 wrapper
- Ros2 cpp package to convert vk_sdk ecal messages into corresponding ros2 messages
## Install

```
# Install ros2

curl -O https://capnproto.org/capnproto-c++-1.0.2.tar.gz
tar zxf capnproto-c++-1.0.2.tar.gz
cd capnproto-c++-1.0.2
./configure
make -j6 check
sudo make install

git clone git@github.com:machinezone/IXWebSocket.git
mkdir build # make a build dir so that you can build out of tree.
cd build
cmake -DUSE_TLS=1 ..
make -j
make install


# Download vk_sdk and run the following
mkdir build && cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Release -DCapnProto_DIR=/usr/local/lib/cmake/CapnProto/ ..
sudo make install
sudo ldconfig
```

## Sample Usage
```
colcon build --packages-select vk_ros2_driver
ros2 launch vk_ros2_driver example.launch.py

# launch with different params file
ros2 launch vk_ros2_driver example.launch.py params_file:=$(ros2 pkg prefix --share vk_ros2_driver)/config/example.yaml
```

- Currently only the following topics are implemented: Images, Disparity, Odometry, Imu, Pointclouds
- odometry frame and camera base_link_frame defined in config yaml
- transforms from base_link to imu / cameras obtained from imu and image msgs
