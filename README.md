openni2-tracker
===============

A ROS Wrapper for the OpenNI2 and NiTE2 Skeleton Tracker

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

3. Clone `openni2_tracker` to your ROS workspace.

    ```bash
    git clone git@github.com:futureneer/openni2-tracker.git
    ```

4. Configure CMake
    In the CMakeLists.txt file inside the `openni2_tracker` package, you will need to change the path where CMake will look for OpenNI2 and NiTE2.  

    Change
    
    ```makefile
    set(OPENNI2_DIR /home/kel/dev/OpenNI2)
    set(NITE2_DIR /home/kel/dev/NiTE-Linux-x64-2.2/)
    ```

