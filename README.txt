Project Name : AntennaDetect-SemiDenseReconstruction
Author :Zuber 
Contact : zbhbupt@126.com  
If you have any question,please contact me.

This project is Antenna detection and semidensemap reconstruction. thare are 6 threads in it,namely, 
main thread : capture img
AntennaDetect thread :  detect Antenna
PointProcessing thread : denose and smoothing the SemiDenseMap
VSLAM thread : provide pose of camera
DepthEstimation thread : do estimation of depth
View thread : display 

There are two modes in this project,

#MODE1  Onlydetect mode
In this mode, system try to detect Antenna but have not detected. In this mode ,VSLAM thread and DepthEstimation thread are not work.once Antenna is detected ,system switchs to #MODE2 .

#MODE2  DepthEstimation mode 
In this mode, all 6 threads is working,which is doing the Antenna detection,VSLAM,estimation of Antenna depth and PointProcessing. Once the VSLAM is lost , system switchs to #MODE1 . 

#1. Project Prerequisites
We have tested the project in **ubuntu 14.04**. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
 Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## BLAS and LAPACK
[BLAS](http://www.netlib.org/blas) and [LAPACK](http://www.netlib.org/lapack) libraries are requiered by g2o (see below). On ubuntu:
```
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
```

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## fast(Included in Thirdparty folder)
it include soure file, this project will used the lib in the folder .if the lib can not be used in you compute, you can build it  again.

## Sophus(Included in Thirdparty folder)
Sophus is a math tools. this project will used the lib in the folder .if the lib can not be used in you compute, you can download from 
https://github.com/strasdat/Sophus.git and build it  again.

## vikit_common(Included in Thirdparty folder)
We use vikit_common for Image coordinate system transformation 
it include soure file, this project will used the lib in the folder .if the lib can not be used in you compute, you can build it  again.

#2 Build the Project

clone the repository 

mkdir build
cd build
cmake ..
make

#Launch the project

 We offer two ways to run this project, there are offline and online respectively.

 #1 Example of offline
    
./mainThread-offline  ~/Config/MVB1-0022.yaml ~/src/VSLAM/Vocabulary/ORBvoc.bin  /media/baohua/media/wireline/02

 
<1>  ~/Config/MVB1-0022.yaml  
Absolute path to configuration file. MVB1-0022.yaml is our cam configuration file, please change it when you run you dataset.

<2> ~/src/VSLAM/Vocabulary/ORBvoc.bin

This file is input file of VSLAM .
<3> /media/baohua/media/wireline/02
Absolute path to dataset. The innermost folder should be this
  			-left
				-left_x.png
			-disp
				-disp_x.png
Note that left_x.png is gray img and disp_x.png is disparity img ,where 0 represent invalue disparity. if you have depth img,just change it in
mainThread-offline.cpp.

#2 Example of online

./mainThread /home/baohua/BX/Config/MVB1-0022.yaml /home/baohua/BX/src/VSLAM/Vocabulary/ORBvoc.bin


<1>  ~/Config/MVB1-0022.yaml  
Absolute path to configuration file. MVB1-0022.yaml is our cam configuration file, please change it when you run you dataset.

<2> ~/src/VSLAM/Vocabulary/ORBvoc.bin

This file is input file of VSLAM .

Note that : change the img capture mode with you camera in mainThread.cpp.










