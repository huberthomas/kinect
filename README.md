# kinect

## Installation

    sudo apt install freenect libusb-1.0-0 
    sudo apt install qt5-default qtcreator

Connect the Kinect V1 and test with the following command

    freenect-glview

If all is working just execute the compiled kinect application.

## Execution

    [-h] Help information. 
    [-o outDir] Output file directory. Default is 'results' in the application directory. 

    During execution the following key events are activated: 

    ESC Close application. 
    SPACE Save a single frame 
    d Switch depth modes. 
    s Start/stop frame saving. 
    a En-/disable auto exposure. 
    w En-/disable white balance. 
    r En-/disable raw color. 

### Hints:

Tested successfully with Ubuntu 16.04 LTS.

Flags for debug or release build:

    cmake -DCMAKE_BUILD_TYPE=Debug
    cmake -DCMAKE_BUILD_TYPE=Release

## Acknowledgement:

Big thanks to https://openkinect.org/wiki/C++OpenCvExample for their great work.
