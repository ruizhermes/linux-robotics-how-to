# Objective:

This repository has an introduction and basic code for Robotics Operating System (ROS). Fundamentals for ROS can be found in the [wiki](https://github.com/ruizhermes/linux-robotics-how-to/wiki) page


**Disclaimer**:
The notes presented in the [wiki](https://github.com/ruizhermes/linux-robotics-how-to/wiki) page are a compilation of different sources such as videos, books, and lectures.


### Linux (Ubuntu) System ROS Configuration

At least the following two installations are required in order to use ROS 

- ### CMake installation 

To install the latest version of **cmake** implement the following steps

1. Download the latest version of cmake from the website
https://cmake.org/download/

1. Unzip the downloaded file
```
  tar tar -xzvf cmake-$version.$build.tar.gz
```
2. Go to the unzipped directory 
```
  cd cmake-$version.$build.tar.gz
```
 
3. Run the executable
```
./bootstrap
```
4. run 
```
make -j4
```
5. run
```
sudo make install
```
6. Check installation version
```
cmake --version
 ```
 
- ### Ubuntu 16.04 ROS Kinetic installation
 
 To install ROS follow this [instructions ](http://wiki.ros.org/kinetic/Installation/Ubuntu) or the link below
 ```
 http://wiki.ros.org/kinetic/Installation/Ubuntu
 ```
