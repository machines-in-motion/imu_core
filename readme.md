imu_core
--------

Realtime and non realtime connection to IMUs.
This package currently support 2 imus:
- imu 3DM_GX3_25 from Lord Sensing System.
- imu 3DM_GX5_25 from Lord Sensing System.

### Installation

#### Standard dependencies

You need:
- [mpi_cmake_modules](https://github.com/machines-in-motion/mpi_cmake_modules)
- [real_time_tools](https://github.com/machines-in-motion/real_time_tools)
- Eigen3
- Boost (filesystem system thread)

The best way to install the dependencies `mpi_cmake_modules` and
`real_time_tools` is to use a ROS2 workspace.
Hence cloning everything in a `workspace/src/` folder and use `colcon` to
build the packages (see below).

For Eigen3 and Boost, they are both available as standard dependencies on
Mac-Os (brew) and Linux/Ubuntu (apt).

#### Download the package

In order to compile this package you will need to also compile
`mpi_cmake_modules` and `real_time_tools` as they do not have binary releases.
In order to do so please clone all sources in a ROS2 workspace folder either
using `git clone` or `treep`:

1. use [treep](https://gitlab.is.tue.mpg.de/amd-clmc/treep) with the
   [treep_machines_in_motion](https://github.com/machines-in-motion/treep_machines_in_motion)
   configuration.
    ```
    mkdir -p ~/devel
    pip3 install treep
    cd ~/devel
    git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
    treep --clone IMU_CORE
    ```

2. use `git clone`  
    ```
    mkdir -p ~/devel/workspace/src
    cd ~/devel/workspace/src
    git clone git@github.com:machines-in-motion/mpi_cmake_modules.git
    git clone git@github.com:machines-in-motion/real_time_tools.git
    git clone git@github.com:machines-in-motion/imu_core.git
    ```

#### Build the package

After downloading all sources you can compile the package using 
[colcon](https://github.com/machines-in-motion/machines-in-motion.github.io/wiki/use_colcon)
```
cd ~/devel/workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Usage

#### Demos

To run the demo plug a valid imu and run:
```
source ~/devel/workspace/install/setup.bash
demo_imu_3DM_GX3_25
```
or
```
source ~/devel/workspace/install/setup.bash
demo_imu_3DM_GX5_25
```

#### API documentation

The API documentation is generated on compile time using a specific CMake
argument:
```
cd ~/devel/workspace/src
colcon build --cmake-args -DGENERATE_DOCUMENTATION=ON
```

[comment]: <> (TODO: *Where to find the last built doc on the internet.*)

### License and Copyrights

See the license.txt file in this repository root folder.
