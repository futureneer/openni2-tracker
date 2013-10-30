openni2_tracker
===============

`openni2_tracker` is a ROS Wrapper for the OpenNI2 and NiTE2 Skeleton Tracker. This is designed as a companion package to the `openni2_camera` package (found [here](https://github.com/ros-drivers/openni2_camera)).  Currently, all this node does is publish TF frames of the current tracked user's joint locations.  I will eventually be adding a user message similar to the old `openni_tracker` ROS package.

** Note:  These instructions only been tested with the Primesense Carmine 1.08 tracker.  There is a way to get this working with the Kinect, using the freenect drivers, but I have not tested that yet.  There is information [here](https://github.com/ros-drivers/openni2_camera) on how to use freenect with OpenNI2.
### Installation
1. Clone the beta OpenNI2 repository from Github:

    ```bash
    git clone git@github.com:OpenNI/OpenNI2.git
    ```
    
    Then, switch to the OpenNI2 directory and build:
    
    ```bash
    cd OpenNI2
    make
    ```

2. Install Nite2 from the OpenNI Website [here](http://www.openni.org/files/nite/?count=1&download=http://www.openni.org/wp-content/uploads/2013/10/NiTE-Linux-x64-2.2.tar1.zip).  Be sure to match the version (x86 or x64) with the version of OpenNI2 you installed above.
You will probably need to create a free account.

    Change directories to the NiTE location and install with 
    
    ```bash
    cd NiTE-Linux-x64-2.2
    sudo ./install.sh
    ```
    Try and run one of the examples in `.../NiTE-Linux-x64-2.2/Samples/Bin/NiTE2`.  If these don't work, then something went wrong with your installation.

3. Clone `openni2_tracker` to your ROS workspace.

    ```bash
    git clone git@github.com:futureneer/openni2-tracker.git
    ```

4. Configure CMake
    In the CMakeLists.txt file inside the `openni2_tracker` package, you will need to change the path where CMake will look for OpenNI2 and NiTE2.  These two lines:
    
    ```makefile
    set(OPENNI2_DIR ~/dev/OpenNI2)
    set(NITE2_DIR ~/dev/NiTE-Linux-x64-2.2/)
    ```
    need to point to the root directories of where you extracted or cloned OpenNI2 and NiTE2.

    
5. Make openni2_tracker

    ```bash
    roscd openni2_tracker
    rosmake
    ```
    
6. Set up NiTE2: Right now, NiTE requires that any executables point to a training sample directory at `.../NiTE-Linux-x64-2.2/Samples/Bin/NiTE2`.  If you run the NiTE sample code, this works fine because those examples are in that same directory.  However, to be able to roslaunch or rosrun openni2_tracker from any current directory, I have created a workaround script `setup_nite.bash`.  This script creates a symbolic link of the NiTE2 directory in your .ros directory (the default working directory for roslaunch / rosrun).  You will need to modify this file so that it points to YOUR NiTE2 and .ros locations.  I would be pleased if anyone has a better solution to this.
7. Run openni2_tracker
    
    ```bash
    roslaunch openni2_tracker tracker.launch
    ```

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
    
### THANKS!
Please let me know if something doesnt work, or if you have suggestions (or feel free to add stuff and send a pull request).
