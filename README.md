# SwagScanner
[![HitCount](http://hits.dwyl.com/{seanngpack}/{swag-scanner}.svg)](http://hits.dwyl.com/{seanngpack}/{swag-scanner})
[![Issues](https://img.shields.io/github/issues-raw/tterb/PlayMusic.svg?maxAge=25000)](https://github.com/seanngpack/swag-scanner/issues)
[![Build Status](https://travis-ci.com/seanngpack/swag-scanner-cpp.svg?branch=master)](https://travis-ci.com/seanngpack/swag-scanner-cpp)
[![MIT License](https://img.shields.io/apm/l/atomic-design-ui.svg?)](https://github.com/seanngpack/swag-scanner/master/LICENSE)  

![Test Image 1](doc/img/swaggg.png)

SwagScanner is a device that can scan your small/medium-sized objects into the virtual world. The system is designed
 and built from the ground up. I designed the software pipeline to be extensible, robust, and fast. SwagScanner is 
 compatible with any depth camera--just verify its minimum scanning distance and acquire the correct mounting hardware 
 and API and you're good to go. You can find more information about the design
  on [my website](https://www.seanngpack.com/swagscanner/).

![sponge](doc/img/sponge.png)  
  

### Status
This is the C++ codebase of Swag Scanner. Everything is highly subject to change.

Hardware: Currently working on physical model complete overhaul. Designing for manufacturing.

Software: Improving GUI, fine tuning registration algos, hand-writing filtering methods 


### Dependencies

<details>
  <summary>Click to see dependencies</summary>

* pcl 1.11
```
$ brew install pcl
```

Or compile from source

```
$ git clone https://github.com/PointCloudLibrary/pcl
$ cd pcl
$ mkdir build
$ cd build
$ cmake ..
$ sudo make install
```

* opencv4
```
brew install opencv
```

* librealsense
```
$ brew install librealsense
```

Or compile from source
```
$ git clone https://github.com/IntelRealSense/librealsense
$ cd librealsense
$ mkdir build
$ cd build
$ cmake ..
$ sudo make install
```

* Qt

```
$ brew install qt
```

* spdlog

```
$ brew install spdlog
```

* feeling-blue-cpp
```
$ git clone https://github.com/seanngpack/feeling-blue-cpp
$ cd feeling-blue
$ mkdir build
$ cd build
$ cmake ..
$ sudo make install
```

</details>

### GUI

SwagScanner has a GUI now!

![GUI image](doc/img/GUI.png)

### CLI Commands

SwagScanner can also be accessed through the commandline.

| --scan          | --calibrate     | --process       | --move       |
|-----------------|-----------------|-----------------|--------------|
| --name (string) | --name (string) | --name (string) | --to (int)   |
| --deg (int)     | --deg (int)     |                 | --by (int)   |
| --rot (int)     | --rot (int)     |                 | --home (int) |

Examples:

Start a scan called "scan1" at 15 degree intervals.

```--scan --name scan1 --deg 15```

Run a calibration with an autogenerated name (because you didn't pass --name) at 8 degree intervals 12 times

```--calibrate --deg 8 --rot 12```


### Authors

* **Sean Ng Pack** - [seanngpack.com](https://www.seanngpack.com)
* **Albert Enyedy** - [github](https://github.com/aenyedy56)


### License

This project is licensed under the MIT License
