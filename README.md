# gnss_comm for ROS2
[**The original README file of this project is here.**](README_old.md)

**All changes made are shown in bold or pre-stated.**

**Authors/Maintainers:** CAO Shaozu (shaozu.cao AT gmail.com)

The *gnss_comm* package contains basic definitions and utility functions for GNSS raw measurement processing. 

**Docker not supported yet**

## 1. Prerequisites

### 1.1 C++11 Compiler
This package requires some features of C++11.

### 1.2 ROS
This package is developed under **[ROS2 Galactic](https://docs.ros.org/en/galactic/index.html)** environment.

### 1.3 Eigen
Our code uses [Eigen 3.3.3](https://gitlab.com/libeigen/eigen/-/archive/3.3.3/eigen-3.3.3.zip) for matrix manipulation. After downloading and unzipping the Eigen source code package, you may install it with the following commands:

```
cd eigen-3.3.3/
mkdir build
cd build
cmake ..
sudo make install
```

### 1.4 Glog
We use google's glog library for message output. If you are using Ubuntu, install it by:
```
sudo apt-get install libgoogle-glog-dev
```
If you are on other OS or just want to build it from source, please follow [these instructions](https://github.com/google/glog#building-glog-with-cmake) to install it.


## 2. **Build gnss_comm_interfaces and gnss_comm (all changed)** 
Clone the repository to your colcon workspace (for example `~/colcon_ws/`):
```
cd ~/colcon_ws/src/
git clone https://github.com/Space-Exploration-UAVTeam/gnss_comm_interfaces.git
git clone https://github.com/Space-Exploration-UAVTeam/gnss_comm.git
```
Then build the package with:
```
cd ~/colcon_ws/
colcon build --symlink-install
source ~/colcon_ws/install/setup.bash
```

## 3. **Docker Not Supported yet**
Any suggestions on Docker will be seen.

## 4. Acknowledgements
Many of the definitions and utility functions in this package are adapted from [RTKLIB](http://www.rtklib.com/).

**All of the definitions and utility functions in this package are adapted from [HKUST-Aerial-Robotics/gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm).**
