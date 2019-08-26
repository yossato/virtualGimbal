# VirtualGimbal
The VirtualGimbal is an electronic stabilize device for videos that were taken by a hand-held camera such as a DSLR. For more information, see [PetaPixel] ( https://petapixel.com/2016/08/11/sd-card-built-gyro-sensor-stabilize-shots/ "PetaPixel").  The VirtualGimbal consists of an SD card and post-processing software. This repository releases the post-processing software, a SD card circuit and a SD card firmware.

## Demo
<https://youtu.be/E9JKbxqoJcY>

## Features
1.Post-processing video stabilization software on PC.  
2.4K video available.  
3.Stabilization completely based on captured angular velocity data.  

## Requirement
Ubuntu 16.04 or later
FFMPEG, OpenCV3, Eigen, boost and some python libraries.

## Usage  
See post_processing_software/README.md  
Download [Example Video](https://drive.google.com/open?id=1_9TezzdYGgDiATJohvIWNb1i1sQY_SVI) and [Angular Velocity data](https://drive.google.com/open?id=1T-ELckV5Ple4VH9Uazb1MwpPFaCNudmW), then put them in ~/vgdataset. A camera calibration file also required to run a demo below so download [Calibration file](https://drive.google.com/open?id=1rUfCPRwqXse2QZHDRD8aU7ulSAihqEy4) then put it in virtualGimbal/post_processing_software/build/camera_descriptions    

Demo:  
```
./pixelwise_stabilizer -i ~/vgdataset/syukugawara/C0003.MP4 -c ILCE-6500 -l SEL1670Z -j records/2019-04-03_07.27.36.json -z 1.3 -o  
```

Here, `i` option is an input video file. `c` is a camera name that is calibrated using a calibrator. `l` is a lens name. `j` is a json file that include an angular velocity record of a gyro sensor. `z` is a zooming ratio. Recommended zooming ratio is 1.3 . `o` option outputs stabilized video.  
Additional option `n` is available to disable preview window. This option speed ups the processing.    
  
  
  
Camera calibration is required before you stabilize your own video. See a [detail information](https://github.com/yossato/virtualGimbal/tree/master/post_processing_software) to do it.


## Install dependencies
### Install system dependencies:
```
sudo apt-get install cmake make 
```

### Install Git:
```
sudo apt install git-all
```

### Install Python Dev:
```
sudo apt-get install python3-dev python3-pip python3-tk python3-lxml python3-six
```

### Install Numpy & Scipy:
```
sudo apt-get install python3-numpy python3-scipy
```

### Install Matplot Library:
```
sudo apt-get install python3-matplotlib
```

### Install OpenCL: See [detail information](https://github.com/intel/compute-runtime/releases)
```
sudo apt install ocl-icd-libopencl1 opencl-headers clinfo ocl-icd-opencl-dev  
cd  
mkdir neo  
cd neo  
wget https://github.com/intel/compute-runtime/releases/download/19.31.13700/intel-gmmlib_19.2.3_amd64.deb  
wget https://github.com/intel/compute-runtime/releases/download/19.31.13700/intel-igc-core_1.0.10-2364_amd64.deb  
wget https://github.com/intel/compute-runtime/releases/download/19.31.13700/intel-igc-opencl_1.0.10-2364_amd64.deb  
wget https://github.com/intel/compute-runtime/releases/download/19.31.13700/intel-opencl_19.31.13700_amd64.deb  
wget https://github.com/intel/compute-runtime/releases/download/19.31.13700/intel-ocloc_19.31.13700_amd64.deb  
sudo dpkg -i *.deb  
```

### Install ffmpeg:  
```
sudo apt install yasm 
cd  
git clone --depth 1 git://source.ffmpeg.org/ffmpeg.git  
cd ffmpeg  
./configure --enable-gpl --enable-x86asm --enable-opencl --enable-ffmpeg --enable-pic --enable-shared --enable-swscale --enable-avresample  
make all -j4  
sudo make install  
sudo ldconfig  
```

### Install cmake:
```
sudo apt-get install cmake
```

### Install OpenCV  
```
cd  
sudo apt-get install build-essential module-assistant libgtk2.0-dev  
sudo m-a prepare  
git clone --depth 1 https://github.com/Itseez/opencv.git  
cd opencv/  
  
mkdir build  
cd build  
cmake .. -DWITH_TBB=ON -DWITH_OPENGL=ON -DWITH_VTK=ON -DWITH_GTK=ON  -DWITH_FFMPEG=ON   
make -j4  
sudo make install  

sudo sh -c "echo /usr/local/lib >> /etc/ld.so.conf.d/opencv.conf"  
sudo sh -c "echo PKG_CONFIG_PATH=\\\$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig >> /etc/bash.bashrc"  
sudo sh -c "echo export PKG_CONFIG_PATH >> /etc/bash.bashrc"  
```

### Install Eigen
```
cd  
wget -O Eigen3.2.10.tar.gz http://bitbucket.org/eigen/eigen/get/3.2.10.tar.gz  
tar zxvf Eigen3.2.10.tar.gz  
cd eigen-*  
mkdir build  
cd build  
cmake ..  
make -j4  
sudo make install  
```

## Build
```
cd
git clone https://github.com/yossato/virtualGimbal.git  
cd virtualGimbal  
mkdir build  
cd build  
cmake ../post_processing_software/ -DCMAKE_BUILD_TYPE=Release  
make -j4  
```

## To start up faster  
I added a program angular_velocity_estimator that generates a JSON file. The file will be saved in same directory of video file after execution.
If the file exists, the pixelwise_stabilizer starts up faster.
To run the program, execute a command like below.
./angular_velocity_estimator ~/vgdataset/syukugawara/C0003.MP4

This command generates C0003.json .

The file contains estimated angular velocity from the video that is used for synchronizing between video and angular velocity measured by the gyro sensor.

## Contribution

## Todo
- Speeding up
- Brushing up deblur quality

## Author
Yoshiaki Sato
