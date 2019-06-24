# kinect

## Installation

    sudo apt install freenect libusb-1.0-0 
    sudo apt install qt5-default qtcreator

Connect the Kinect V1 and test with the following command

    freenect-glview

If all is working just execute the compiled kinect application.

### Hints:

Flags for debug or release build:

cmake -DCMAKE_BUILD_TYPE=Debug
cmake -DCMAKE_BUILD_TYPE=Release

## Acknowledgement:

Big thanks to https://openkinect.org/wiki/C++OpenCvExample for their great work.
