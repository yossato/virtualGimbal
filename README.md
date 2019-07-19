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
FFMPEG and OpenCV3

## Usage  
See post_processing_software/README.md  
Download [Example Video](https://drive.google.com/uc?export=download&id=0B9nCHvB3LdAxZWNKdmdxMTFzam8) and [Angular Velocity data](https://drive.google.com/uc?export=download&id=0B9nCHvB3LdAxTHB1dk0zMkZWbDQ), then put them in ~/vgdataset.

Demo:  
```
./rolling_shutter_parameter_estimator -i ~/vgdataset/syukugawara/C0003.MP4 -c ILCE-6500 -l SEL1670Z -j records/2019-04-03_07.27.36.json -z 1.3  
```


Here, `i` option is an input video file. `c` is a camera name that is calibrated using a calibrator. `l` is a lens name. `j` is a json file that include an angular velocity record of a gyro sensor. `z` is a zooming ratio. Recommended zooming ratio is 1.3 . 

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

### Install lame:  
```
cd  
wget -O lame-3.99.5.tar.gz http://downloads.sourceforge.net/project/lame/lame/3.99/lame-3.99.5.tar.gz?r=http%3A%2F%2Fsourceforge.net%2Fprojects%2Flame%2Ffiles%2Flame%2F3.99%2F&ts=1438787999&use_mirror=jaist  
tar zxvf lame-3.99.5.tar.gz  
cd lame-3.99.5  
./configure  
make -j4  
sudo make install  
sudo ldconfig
```

### Install ffmpeg:  
```
cd  
git clone --depth 1 git://source.ffmpeg.org/ffmpeg.git  
cd ffmpeg  
./configure --enable-gpl --enable-libmp3lame --disable-yasm --enable-ffmpeg --enable-ffmpeg --enable-pic --enable-shared --enable-swscale --enable-avresample  
make all -j4  
sudo make install  
sudo ldconfig  
```

### Install QT:  
```
sudo apt-get install qt5-default
```

### Install cmake:
```
sudo apt-get install cmake
```

### Install OpenCV  
```
cd  
sudo apt-get install build-essential module-assistant  
sudo m-a prepare  
git clone --depth 1 https://github.com/Itseez/opencv.git  
cd opencv/  
  
mkdir build  
cd build  
cmake .. -DWITH_TBB=ON -DWITH_QT=ON -DWITH_OPENGL=ON -DWITH_VTK=ON  
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

### Install Boost
```
sudo apt-get install libboost-dev
```
## Build
```
cd
git clone https://github.com/yossato/macadamia.git  
cd macadamia  
mkdir build  
cd build  
cmake ../post_processing_software/  
make -j4  
```

## Contribution

## Todo/Issues/Wishlist

## Author
Yoshiaki Sato
