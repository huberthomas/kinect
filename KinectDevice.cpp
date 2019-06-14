#include "KinectDevice.h"
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"

/**
 * @brief Kinect device class.
 * @param ctx Context information.
 * @param index Index of the kinect device.
 */
KinectDevice::KinectDevice(freenect_context *ctx, int index)
    : Freenect::FreenectDevice(ctx, index), _bufferDepth(FREENECT_DEPTH_11BIT),
      _bufferRgb(FREENECT_VIDEO_RGB), _gamma(2048), _getNewRgbFrame(false),
      _getNewDepthFrame(false), _depthMat(cv::Size(640,480), CV_16UC1),
      _rgbMat(cv::Size(640,480), CV_8UC3, cv::Scalar(0)) {

    for( unsigned int i = 0 ; i < 2048 ; i++) {
        float v = i/2048.0;
        v = std::pow(v, 3) * 6;
        _gamma[i] = v * 6 * 256;
    }
}

/**
 * @brief RGB video callback. Do never call directly.
 * @param rgb RGB image.
 * @param timestamp Capturing timestamp.
 */
void KinectDevice::VideoCallback(void *rgb, uint32_t timestamp) {
    std::cout << "RGB callback " << timestamp << std::endl;
    _rgbMutex.lock();
    uint8_t* img = static_cast<uint8_t*>(rgb);
    _rgbMat.data = img;
    _rgbTimestamp = timestamp;
    _getNewRgbFrame = true;
    _rgbMutex.unlock();
}

/**
 * @brief Depth video stream callback. Do never call directly.
 * @param depth
 * @param timestamp Capturing timestamp.
 */
void KinectDevice::DepthCallback(void *depth, uint32_t timestamp) {
    std::cout << "Depth callback " << timestamp << std::endl;
    _depthMutex.lock();
    uint16_t* img = static_cast<uint16_t*>(depth);
    _depthMat.data = (uchar*)img;
    _depthTimestamp = timestamp;
    _getNewDepthFrame = true;
    _depthMutex.unlock();
}

/**
 * @brief Get RGB image.
 * @param output
 * @param timestamp Capturing timestamp.
 * @return
 */
bool KinectDevice::getVideo(cv::Mat &output, uint32_t &timestamp) {
    _rgbMutex.lock();
    if(_getNewRgbFrame) {
        cv::cvtColor(_rgbMat, output, CV_RGB2BGR);
        timestamp = _rgbTimestamp;
        _getNewRgbFrame = false;
        _rgbMutex.unlock();
        return true;
    } else {
        _rgbMutex.unlock();
        return false;
    }
}

/**
 * @brief Get depth image.
 * @param output Depth image.
 * @param timestamp Capturing timestamp.
 * @return
 */
bool KinectDevice::getDepth(cv::Mat &output, uint32_t &timestamp) {
    _depthMutex.lock();
    if(_getNewDepthFrame) {
        _depthMat.copyTo(output);
        timestamp = _depthTimestamp;
        _getNewDepthFrame = false;
        _depthMutex.unlock();
        return true;
    } else {
        _depthMutex.unlock();
        return false;
    }
}


