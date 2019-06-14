#include <iostream>
#include <highgui.h>
#include <unistd.h>
#include <chrono>
#include "KinectDevice.h"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"

using namespace cv;
using namespace std;

/**
 * @brief Main entry.
 * @param argc Argument counter.
 * @param argv Argument values.
 * @return Exit code.
 */
int main(int argc, char **argv) {
    bool die(false);

    //	QString date = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh:mm:ss");
    string prefix("");
    string suffix(".png");
    string rgbFilePath("results/rgb/");
    string depthFilePath("results/depth/");

    int iter(0);

    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));

    uint32_t rgbTimestamp;
    uint32_t depthTimestamp;
    // The next two lines must be changed as Freenect::Freenect
    // isn't a template but the method createDevice:
    // Freenect::Freenect<MyFreenectDevice> freenect;
    // MyFreenectDevice& device = freenect.createDevice(0);
    // by these two lines:

    Freenect::Freenect freenect;

    KinectDevice &device = freenect.createDevice<KinectDevice>(0);

    //
    // device.setFlag(FREENECT_RAW_COLOR, FREENECT_OFF);

    // freenect_set_flag((freenect_device*)&device, FREENECT_AUTO_EXPOSURE, FREENECT_OFF);
    // freenect_set_flag(device, FREENECT_AUTO_WHITE_BALANCE, FREENECT_OFF);
    // freenect_set_flag(device, FREENECT_RAW_COLOR, FREENECT_OFF);

    namedWindow("rgb",CV_WINDOW_AUTOSIZE);
    namedWindow("depth",CV_WINDOW_AUTOSIZE);
    device.startVideo();
    device.startDepth();

    bool autoExposure = true;
    bool whiteBalance = true;
    bool rawColor = true;
    bool save = false;
    // const auto p0 = std::chrono::time_point<std::chrono::system_clock>{};
    //auto start = std::chrono::duration_cast<std::chrono::seconds>(p0.time_since_epoch()).count();
    // auto start = std::chrono::high_resolution_clock::now();

    while (!die) {

        device.getVideo(rgbMat, rgbTimestamp);
        device.getDepth(depthMat, depthTimestamp);

        // const auto p1 = std::chrono::system_clock::now();
        // auto finish = std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count();
        // std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
        // std::cout << elapsed.count() << " " << finish << " " << rgbTimestamp << std::endl;

        cv::imshow("rgb", rgbMat);
        depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
        cv::applyColorMap(depthf, depthf, COLORMAP_JET);
        cv::imshow("depth",depthf);

        char k = cvWaitKey(5);
        // see http://www.asciitable.com/ key codes
        if( k == 27 ){
            // ESC
            cvDestroyWindow("rgb");
            cvDestroyWindow("depth");
            break;
        }
        if(k == 97) {
            // a
            std::cout << "FREENECT_AUTO_EXPOSURE: " << autoExposure << std::endl;
            autoExposure = !autoExposure;
            device.setFlag(FREENECT_AUTO_EXPOSURE, autoExposure ? FREENECT_ON : FREENECT_OFF);
            usleep(1000 * 1000);
        }
        if(k == 119) {
            // w
            std::cout << "FREENECT_AUTO_WHITE_BALANCE: " << whiteBalance << std::endl;
            whiteBalance = !whiteBalance;
            device.setFlag(FREENECT_AUTO_WHITE_BALANCE, whiteBalance ? FREENECT_ON : FREENECT_OFF);
            usleep(1000 * 1000);
        }
        if(k == 114) {
            // r
            std::cout << "FREENECT_RAW_COLOR: " << rawColor << std::endl;
            rawColor = !rawColor;
            device.setFlag(FREENECT_RAW_COLOR, rawColor ? FREENECT_ON : FREENECT_OFF);
            usleep(1000 * 1000);
        }
        if( k == 115 ) {
            // s
            save = !save;
        }
        if (save) {
            std::ostringstream rgbFile;
            std::ostringstream depthFile;
            rgbFile << rgbFilePath << prefix << rgbTimestamp << suffix;
            depthFile << depthFilePath << prefix << depthTimestamp << suffix;
            cv::imwrite(rgbFile.str(), rgbMat);
            cv::imwrite(depthFile.str(), depthMat);
        }

        if(iter >= 1000) break;
        iter++;
    }

    device.stopVideo();
    device.stopDepth();
    return 0;
}
