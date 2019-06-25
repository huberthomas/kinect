# kinect

This is a small appication to capture RGB and depth data from the XBOX360 Kinect device. 
It supports some controls over the device. For this see the execution subsection. The stream
can be saved on your hard disk, the default directory structure is

    kinect
    - results
    | - capturing_start_timestamp
    | | - rgb
    | | | capturing_timestamp_in_ms_from_time_epoch.png
    | | - depth
    | | | capturing_timestamp_in_ms_from_time_epoch.png
    | | | rgb_depth.txt

The 'rgb_depth.txt' file contains a list of associated RGB and depth image files. They are linked
by the shortest timestamp difference, files over 200ms difference are skipped.

## Installation

    sudo apt install freenect libusb-1.0-0 
    sudo apt install qt5-default qtcreator

Connect the Kinect V1 and test with the following command

    freenect-glview

If all is working just execute the compiled kinect application.

## Execution

    > ./kinect
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
