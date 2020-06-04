# Linux (Ubuntu) Installations

### CMake installation 

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
 
 ## Installing ROS Kinetic in Ubuntu 16.04
 
 To install ROS follow the instructions at 
 ```
 http://wiki.ros.org/kinetic/Installation/Ubuntu
 ```
