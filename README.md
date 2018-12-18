openni2_tracker
===============

This work is developed from [here](https://github.com/futureneer/openni2-tracker), which is based on out-dated rosbuild system. I made it support catkin system, and write a more detailed README to make user can build their development environment step by step.

`openni2_tracker` is a ROS Wrapper for the OpenNI2 and NiTE2 Skeleton Tracker. This is designed as a companion package to the `openni2_camera` package (found [here](https://github.com/ros-drivers/openni2_camera)).  Currently, all this node does is publish TF frames of the current tracked user's joint locations.

**Note**:  These instructions only been tested with ASUS Xtion Pro Live in ROS-kinetic, Ubuntu 16.04. You can directly access to the **docker image** file which support all software development environment within OpenGL visualization at [here](https://cloud.docker.com/repository/docker/bluebirdpp/openni2/tags), or just run the following command to get the docker image:
   ```bash
   docker pull bluebirdpp/openni2:latest
   ```

### Installation
1. Install OpenNI2:
    
    ```bash
    sudo apt install git libusb-1.0-0-dev libudev-dev
    sudo apt install openjdk-8-jdk  # for xenial; openjdk-6-jdk for trusty; if not using other java version.
    sudo apt install freeglut3-dev
    cd  # go home
    mkdir -p src && cd src  # create $HOME/src if it doesn't exist; then, enter it
    git clone https://github.com/occipital/OpenNI2.git  # We used to have a fork off 6857677beee08e264fc5aeecb1adf647a7d616ab with working copy of Xtion Pro Live OpenNI2 driver.
    cd OpenNI2
    make -j$(nproc)  # compile
    sudo ln -s $PWD/Bin/x64-Release/libOpenNI2.so /usr/local/lib/  # $PWD should be /yourPathTo/OpenNI2
    sudo ln -s $PWD/Bin/x64-Release/OpenNI2/ /usr/local/lib/  # $PWD should be /yourPathTo/OpenNI2
    sudo ln -s $PWD/Include /usr/local/include/OpenNI2  # $PWD should be /yourPathTo/OpenNI2
    sudo ldconfig
    ```
    
2. Install ASUS Xtion Pro Live OpenNI driver:
    
    ```bash
    sudo apt install libopenni-sensor-primesense0
    ```
    
3. Install NiTE2.2:
    
    ```bash
    cd  # go home
    mkdir -p src && cd src  # create $HOME/src if it doesn't exist; then, enter it
    wget https://sourceforge.net/projects/roboticslab/files/External/nite/NiTE-Linux-x64-2.2.tar.bz2
    tar xvf NiTE-Linux-x64-2.2.tar.bz2
    sudo ln -s $PWD/NiTE-Linux-x64-2.2/Redist/libNiTE2.so /usr/local/lib/  # $PWD should be /yourPathTo/NiTE-Linux-x64-2.2/..
    sudo ln -s $PWD/NiTE-Linux-x64-2.2/Include /usr/local/include/NiTE-Linux-x64-2.2  # $PWD should be /yourPathTo/NiTE-Linux-x64-2.2/..
    sudo ldconfig
    ```
    
    To make sure your NiTE2.2 can run successfully, you need to run some NiTE built-in binary file:
    
    ```bash
    cd ~/src/NiTE-Linux-x64-2.2/Samples/Bin
    ./UserViewer
    ```
    If you got a depth images stream, that means your NiTE work as expected. If you got "no device found" error, you may need to do this:
    
    ```bash
    sudo ln -s /lib/x86_64-linux-gnu/libudev.so.1.6.4 /lib/x86_64-linux-gnu/libudev.so.0
    ````
    
4. Install orocos_kdl
    ```bash
    cd ~/src
    git clone https://github.com/orocos/orocos_kinematics_dynamics.git
    sudo apt-get -qq update
    sudo apt-get install libeigen3-dev libcppunit-dev
    cd orocos_kinematics_dynamics/orocos_kdl
    mkdir build
    cd build
    cmake -DENABLE_TESTS:BOOL=ON -DCMAKE_CXX_FLAGS:STRING="-Wall" -DCMAKE_BUILD_TYPE=${OROCOS_KDL_BUILD_TYPE} ./..
    # compile and install orocos_kdl
    make -j`nproc`
    sudo make install
    ````

5. Clone `openni2_tracker` to your ROS workspace:
    ```bash
    cd ~/catkin_ws/src
    git clone git@github.com:msr-peng/openni2_tracker.git
    ```
    
6. Configure CMake:
    
    If you followed Step 1 and Step 2 **strictlly**, then you needn't do anything. Otherwise, you need to modify `CMakeList.txt` in `openni2_tracker` package to make your project can find OpenNI2 and NiTE2.2
    
7. Make openni2_tracker:

    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
    
8. Set up NiTE2:
    
    Right now, NiTE requires that any executables point to a training sample directory at `.../NiTE-Linux-x64-2.2/Samples/Bin/NiTE2`.  If you run the NiTE sample code, this works fine because those examples are in that same directory.  However, to be able to roslaunch or rosrun openni2_tracker from any current directory, I have created a workaround script `setup_nite.bash`.  This script creates a symbolic link of the NiTE2 directory in your .ros directory (the default working directory for roslaunch / rosrun).  You will need to modify this file so that it points to YOUR NiTE2 and .ros locations.
    By default, you need do this:
    ```bash
    cd ~/catkin_ws/src/openni2_tracker
    ./setup_nite.bash
    ```
    I would be pleased if anyone has a better solution to this.
    
9. Run openni2_tracker:
    
    ```bash
    roslaunch openni2_tracker tracker.launch
    ```
    
    Then you can visualize the skeleton tracking results on `Rviz`:
    
    ```bash
    rosrun rviz rviz -f /tracker_depth_frame
    ```

    Finally in `Rivz`, add `TF` to visualize the skeleton keypoints coordinates.
    
    In the lauch file, you can rename both the tracker name and the tracker's relative frame.  I have included a static publisher that aligns the tracker frame to the world frame, approximately 1.25m off the floor.
    
    ```xml
    <!-- openni2_tracker Launch File -->
    <launch>
    
      <arg name="tracker_name" default="tracker" />
      
      <node name="tracker" output="screen" pkg="openni2_tracker" type="tracker" >
        <param name="tf_prefix" value="$(arg tracker_name)" />
        <param name="relative_frame" value="/$(arg tracker_name)_depth_frame" />
      </node>
    
      <!-- TF Static Transforms to World -->
      <node pkg="tf" type="static_transform_publisher" name="world_to_tracker" args=" 0 0 1.25 1.5707 0 1.7707  /world /$(arg tracker_name)_depth_frame 100"/> 
    
    </launch>
    ```
    
    Currently, this node will broadcast TF frames of the joints of any user being tracked by the tracker.  The frame names are based on the tracker name, currently `/tracker/user_x/joint_name`
