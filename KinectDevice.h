#pragma once
#ifndef KINECT_DEVICE_H
#define KINECT_DEVICE_H

#include <mutex>
#include <chrono>

#include "libfreenect.hpp"
#include "opencv2/core/core.hpp"

class KinectDevice : public Freenect::FreenectDevice {
public:
    KinectDevice(freenect_context *ctx, int index);

    void VideoCallback(void* rgb, uint32_t timestamp) override;
    void DepthCallback(void* depth, uint32_t timestamp) override;
    void getVideo(cv::Mat& output, int64_t &timestamp) noexcept;
    void getDepth(cv::Mat& output, int64_t& timestamp) noexcept;

private:
    std::vector<uint8_t> _bufferDepth{FREENECT_DEPTH_11BIT};
    std::vector<uint8_t> _bufferRgb{FREENECT_VIDEO_RGB};
    cv::Mat _depthMat{cv::Mat(cv::Size(640,480), CV_16UC1)};
    cv::Mat _rgbMat{cv::Mat(cv::Size(640,480), CV_8UC3, cv::Scalar(0))};
    std::mutex _rgbMutex;
    std::mutex _depthMutex;
    int64_t _rgbTimestamp;
    int64_t _depthTimestamp;
    bool _getNewRgbFrame{false};
    bool _getNewDepthFrame{false};
};

#endif // KINECT_DEVICE_H
