#include "KinectDevice.h"
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include <chrono>

/**
 * @brief Kinect device class.
 * @param ctx Context information.
 * @param index Index of the kinect device.
 */
KinectDevice::KinectDevice(freenect_context *ctx, int index)
    : Freenect::FreenectDevice(ctx, index)
{
}

/**
 * @brief RGB video callback. Do never call directly.
 * @param rgb RGB image frame.
 * @param timestamp Capturing timestamp.
 */
void KinectDevice::VideoCallback(void *rgb, uint32_t timestamp) {
    _rgbMutex.lock();

    auto currentTime = std::chrono::high_resolution_clock::now().time_since_epoch();
    auto current = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime).count();

    uint8_t* img = static_cast<uint8_t*>(rgb);

    _rgbMat.data = img;
    _rgbTimestamp = current;
    _getNewRgbFrame = true;
    _rgbMutex.unlock();
}

/**
 * @brief Depth video stream callback. Do never call directly.
 * @param depth Depth image frame.
 * @param timestamp Capturing timestamp.
 */
void KinectDevice::DepthCallback(void *depth, uint32_t timestamp) {
    _depthMutex.lock();

    auto currentTime = std::chrono::high_resolution_clock::now().time_since_epoch();
    auto current = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime).count();

    uint16_t * img = static_cast<uint16_t*>(depth);

    _depthMat.data = (uchar*)img;

    _depthTimestamp = current;
    _getNewDepthFrame = true;
    _depthMutex.unlock();
}

/**
 * @brief Get RGB image.
 * @param output RGB image frame.
 * @param timestamp Capturing timestamp.
 */
void KinectDevice::getVideo(cv::Mat &output, int64_t &timestamp) noexcept
{
    _rgbMutex.lock();
    if(_getNewRgbFrame) {
        cv::cvtColor(_rgbMat, output, CV_RGB2BGR);

        timestamp = _rgbTimestamp;

        _getNewRgbFrame = false;
        _rgbMutex.unlock();
    } else {
        _rgbMutex.unlock();
    }
}

/**
 * @brief Get depth image.
 * @param output Depth image frame.
 * @param timestamp Capturing timestamp.
 */
void KinectDevice::getDepth(cv::Mat &output, int64_t &timestamp) noexcept
{
    _depthMutex.lock();
    if(_getNewDepthFrame) {
        _depthMat.copyTo(output);

        timestamp = _depthTimestamp;

        _getNewDepthFrame = false;
        _depthMutex.unlock();
    } else {
        _depthMutex.unlock();
    }
}


