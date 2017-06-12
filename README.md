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

## Install

## Contribution

## Licence



## Author
Yoshiaki Sato
