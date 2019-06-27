#pragma once
#ifndef KINECT_HANDLER_H
#define KINECT_HANDLER_H
#include <string>
#include <vector>

#include "KinectDevice.h"

class KinectHandler {
public:
    KinectHandler() = default;

    void startCapturing();
    void stopCapturing();

    void setOutputDir(const std::string &outputDir);

    void associateFiles(const std::string &rgbDir,
                        const std::__cxx11::string &depthDir,
                        const std::string &resultDir,
                        const bool &cleanUp = true,
                        const int &maxDifferenceMs = 200);

private:
    std::string _prefix{""};
    std::string _suffix{""};
    std::string _extension{"png"};
    std::string _outputDir{""};
    std::string _outputFile{"rgb_depth.txt"};

    bool _isRunning{false};
    bool _enableAutoExposure{true};
    bool _enableWhiteBalance{true};
    bool _enableRawColor{true};

    KinectDevice * _device{nullptr};
    void displayResults(const cv::Mat &rgb, const cv::Mat &depth) noexcept;
    void createDirectory(const std::string &dirPath);
    bool checkDir(std::string &dir) noexcept;
    std::string extractFileName(const std::string &file);
    std::vector<std::string> getFiles(const std::string &dir, const std::string &filter) noexcept;
    void init();
    void setDepthMode(const short &depthMode);
    std::string getCurrentDateTime() noexcept;
};

#endif
