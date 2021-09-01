# amcl3d_test

### Acknowledgements

This just a test demo. And maintenance may not be updated.

See the [blog](https://blog.csdn.net/u012700322/article/details/120032339) for the algorithm flow.

### Overview

This is a package is a **"Adaptive Monte-Carlo Localization in 3D"**.

It is a particle filter that estimates the localization of a robot moving in a 3D environment.

It takes information from an odometry, 3D laser point-clouds.


The amcl3d package has been tested under [ROS] Kinetic and Ubuntu 16.04.

#### Detailed Description




### Demostration Image

![image](https://github.com/chengwei0427/amcl3d_test/blob/master/doc/pf_demo.png)

### Installation

#### Building from Source

##### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

     cd catkin_workspace/src
     git clone https://github.com/chengwei0427/amcl3d_test.git
     cd ../
     catkin build


