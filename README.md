# 4d_association
Code for cvpr2020 "4D Association Graph for Realtime Multi-person Motion Capture Using Multiple Video Cameras"

### Introduction
Our work [4D Association Graph](http://www.liuyebin.com/4dassociation) contributes a novel realtime multi-personmotion capture algorithm using multiview video inputs. Due to the heavy occlusions and closely interacting motions in each view, joint optimization on the multiview images and multiple temporal frames is indispensable, which brings up the essential challenge of realtime efﬁciency. Our contributions include:
+ 4D Association Graph Representation: unify per-view parsing, cross-view matching, and temporal tracking into a single optimization framework that each dimension can be treated equally and simultaneously. Existing association methods consider only parts of these associations, or simply operate them in a sequential manner, which reduce the quality and efficiency of motion reconstruction. 
+ Robust Realtime Motion Capture System: propose a 4D limb bundle parsing solver based on heuristic searching to enable realtime reconstruction(30fps).
+ Multi-person 3D Skeleton Dataset: multi-view RGB video with aligned high-quality 3D skeletons from optitrack, mainly focusing on close interactions and challenging motion.

### Build Instructions (Using CMake)
This project uses CMake as the build script, which has been compiled and tested under the following development environment:
+ Visual Studio 2017 (x64)
+ Visual Studio 2019 (x64)
+ GCC version 9.3.0+ (Ubuntu 20.04 x64)
The project requires the following components: [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), [jsoncpp](https://github.com/open-source-parsers/jsoncpp) and [opencv(4.2+)](https://opencv.org/).

#### How to build on Windows (Visual Studio 2017/2019 x64)
###### 1. Download code.
Use any method you are familiar with to get the code.
###### 2. Configure OpenCV environment.
1. Get OpenCV Windows package (4.2 or 4.4 version) from https://opencv.org/releases/, and unzip it a place, such as 'D:\Software\OpenCV\'.'
2. Create the following environment variables:
```
ENV: OPENCV_DIR
VALUE: D:\Software\OpenCV\build
```
```
ENV: OpenCV_BIN_DIR
VALUE: %OPENCV_DIR%\x64\vc14\bin
```
3. Add the following value to 'path' envirorment variable:
```
%OpenCV_BIN_DIR%
```
###### 3. Configure other dependent libraries.
Obtain DependentLibraries.zip and unzip it into the repository folder. Overwrite the original include and library directories.
You can get DependentLibraries.zip from https://pan.baidu.com/s/1WMPvqZLmu64ZE35wMHHqgw code: jbid
###### 4. Generate Visual Studio project files.
For Visual Studio 2017 (x64)
```
build\vs2017-x64\make-solutions.bat
```
For Visual Studio 2019 (x64)
```
build\vs2019-x64\make-solutions.bat
```
###### 5. Compile the project.
Double-click 'BaseProject.sln' directly to open the project for compilation. And you can use both release and debug mode.

#### How to build on Linux (Ubuntu 20.04 x64)
1. Install dependent components.
```
sudo apt-get install git gitg vim
sudo apt-get install cmake*
sudo apt-get install libopencv-*
sudo apt-get install libjsoncpp-dev 
sudo apt-get install libeigen3-dev 
```
2. Download code and compile.
```
git clone https://github.com/PoeticFlower/4d_association.git 4d_association
cd 4d_association/
cd build/
cd linux-native/
./make-Makefiles.sh
make
cd four_d_association/
./mocap
./evaluate_shelf
```

### Getting Started

Our project run on visual studio 2017(c++ 17), with requriements [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), [jsoncpp](https://github.com/open-source-parsers/jsoncpp) and [opencv(4.2)](https://opencv.org/). 

You can download our pre-built bin file on

```
 https://pan.baidu.com/s/184RRDghnwiSCmEbxneyamQ  4enh
```

 Then Unzip the dependency file in `C:\cppmodule` or other place, and check the `.props` file to make sure compiler can find the correct include path and lib path. Moreover, you need to copy the dll file of dependency to exe directory or add them to system environment.  
 Finally, you will choose platform `release` and `x64` to compile and run our demo.

### Citation

```
Yuxiang Zhang, Liang An, Tao Yu, xiu Li, Kun Li, Yebin Liu. "4D Association Graph for Realtime Multi-person Motion Capture Using Multiple Video Cameras". arXiv 2020

@InProceedings{20204DAssociation,
  author = {Zhang, Yuxiang and An, Liang and Yu, Tao and Li, xiu and Li, Kun and Liu, Yebin},
  title = {4D Association Graph for Realtime Multi-person Motion Capture Using Multiple Video Cameras},
  booktitle = {IEEE International Conference on Computer Vision and Pattern Recognition, (CVPR)},
  year={2020},
}
```
