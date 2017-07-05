# virtualGimbal
virtualGimbal is an electronic stabilize device for videos that were taken by hand held camera. (I.e. DSLR)  
For more information, see [PetaPixel]( https://petapixel.com/2016/08/11/sd-card-built-gyro-sensor-stabilize-shots/ "PetaPixel").  
virtualGimbal consists of an SD card and a Post processing software. This repository releases the post processing software.

## Demo
<https://youtu.be/E9JKbxqoJcY>

## Features 
1.Post processing video stabilization software on PC.  
2.Real-time up to 1920 x 1080 at 30 fps.  
3.Stabilization based on angular velocity data.  

## Requirement
Ubuntu 16.04  
OpenCV 3.1  
FFMpeg  
OpenGL 3.3 or later

## Usage
Demo:  
`./virtualGimbal -i ~/vgdataset/guam.mts -c ~/vgdataset/anglarVelocity.csv`  
  
Generating stabilized video:  
`./virtualGimbal -i ~/vgdataset/guam.mts -c ~/vgdataset/anglarVelocity.csv -o`  

## Install dependencies
### Install system dependencies:  
`sudo apt-get install cmake make g++ libx11-dev libxi-dev libgl1-mesa-dev libglu1-mesa-dev libxrandr-dev libxext-dev`  
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
`sudo apt-get install qt5-default`  
### Install cmake:
`sudo apt-get install cmake`
  
### Install OpenCV  
```cd  
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

### Install Boost
`sudo apt-get install libboost-dev`

## Build
```git clone http://github.com/yossato/virtualGimbal  
cd virtualGimbal  
mkdir build  
cd build  
cmake ..  
make  
```
## Contribution

## Licence
virtualGimbal is licensed under the GNU General Public License Version 2 (GPLv2), see http://www.gnu.org/licenses/gpl.html.  
For commercial purposes, we also offer a professional version under different licencing terms. Please contact me if you need.  


## Author
Yoshiaki Sato
