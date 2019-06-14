#pragma once
#ifndef KINECT_DEVICE_H
#define KINECT_DEVICE_H

#include "libfreenect.hpp"
#include "opencv2/core/core.hpp"
#include <QMutex>

class KinectDevice : public Freenect::FreenectDevice {
public:
    KinectDevice(freenect_context *ctx, int index);

    void VideoCallback(void* rgb, uint32_t timestamp) override;
    void DepthCallback(void* depth, uint32_t timestamp) override;
    bool getVideo(cv::Mat& output, uint32_t& timestamp);
    bool getDepth(cv::Mat& output, uint32_t& timestamp);

private:
    std::vector<uint8_t> _bufferDepth;
    std::vector<uint8_t> _bufferRgb;
    std::vector<uint16_t> _gamma;
    cv::Mat _depthMat;
    cv::Mat _rgbMat;
    QMutex _rgbMutex;
    QMutex _depthMutex;
    uint32_t _rgbTimestamp;
    uint32_t _depthTimestamp;
    bool _getNewRgbFrame;
    bool _getNewDepthFrame;
};

#endif // KINECT_DEVICE_H
