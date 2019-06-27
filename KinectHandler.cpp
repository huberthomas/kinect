#include "KinectHandler.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <map>
#include <stdio.h>

#include <QDir>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"

using namespace std;

#define cvWindowTitleRgb "RGB"
#define cvWindowTitleDepth "Depth"

/**
 * @brief Start capturing image and depth frames from the Kinect device.
 */
void KinectHandler::startCapturing() {
    _isRunning = true;

    const string separator = "/";
    string resDir{""};
    string rgbDir{""};
    string depthDir{""};

    int64_t rgbTimestamp{0};
    int64_t depthTimestamp{0};

    cv::Mat depthMat(cv::Size(640,480), CV_16UC1);
    cv::Mat rgbMat(cv::Size(640,480), CV_8UC3, cv::Scalar(0));

    Freenect::Freenect freenect;
    short depthMode = 4;

    if(_device) {
        _device->stopVideo();
        _device->stopDepth();
    }

    _device = &freenect.createDevice<KinectDevice>(0);

    setDepthMode(depthMode);

    _device->startVideo();
    _device->startDepth();

    bool save = false;
    bool resultDirectoriesCreated = false;

    cv::namedWindow(cvWindowTitleRgb, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(cvWindowTitleDepth, CV_WINDOW_AUTOSIZE);

    while(_isRunning) {
        _device->getVideo(rgbMat, rgbTimestamp);
        _device->getDepth(depthMat, depthTimestamp);

        displayResults(rgbMat, depthMat);

        char k = cvWaitKey(5);
        // see http://www.asciitable.com key codes
        if( k == 27 ){
            // ESC
            cout << "quit application" << endl;
            break;
        }
        if(k == 97) {
            // a
            _enableAutoExposure = !_enableAutoExposure;
            _device->setFlag(FREENECT_AUTO_EXPOSURE, _enableAutoExposure);

            cout << "FREENECT_AUTO_EXPOSURE: " << (_enableAutoExposure ? "on" : "off") << endl;
        }
        if(k == 119) {
            // w
            _enableWhiteBalance = !_enableWhiteBalance;
            _device->setFlag(FREENECT_AUTO_WHITE_BALANCE, _enableWhiteBalance);

            cout << "FREENECT_AUTO_WHITE_BALANCE: " << (_enableWhiteBalance ? "on" : "off") << endl;
        }
        if(k == 114) {
            // r
            _enableRawColor = !_enableRawColor;
            _device->setFlag(FREENECT_RAW_COLOR, _enableRawColor);

            cout << "FREENECT_RAW_COLOR: " << (_enableRawColor ? "on" : "off") << endl;
        }
        if(k == 115) {
            // s
            save = !save;

            if(save) {
                resultDirectoriesCreated = false;
            }
            if(rgbDir.size() > 0 && depthDir.size() > 0 && resDir.size() > 0){
                associateFiles(rgbDir, depthDir, resDir);
            }

            cout << (save ? "start saving stream" : "stop saving stream") << endl;
        }
        if(k == 100) {
            // d
            depthMode++;

            if(depthMode % 6 == 0) {
                depthMode = 0;
            }

            setDepthMode(depthMode);
        }
        if(k == 32 || save) {
            // space
            if(!resultDirectoriesCreated) {
                string subOutputDir = getCurrentDateTime();

                resDir = _outputDir + "results" + separator + subOutputDir + separator;
                rgbDir = resDir + "rgb" + separator;
                depthDir = resDir + "depth" + separator;

                createDirectory(rgbDir);
                createDirectory(depthDir);

                resultDirectoriesCreated = true;
            }

            cout << "saving rgb: " << _prefix << rgbTimestamp << _suffix << "." << _extension;
            cout << " depth: " << _prefix << depthTimestamp << _suffix << "." << _extension << endl;

            ostringstream rgbFile;
            ostringstream depthFile;
            rgbFile << rgbDir << _prefix << rgbTimestamp << _suffix << "." << _extension;
            depthFile << depthDir << _prefix << depthTimestamp << _suffix << "." << _extension;

            cv::imwrite(rgbFile.str(), rgbMat);
            cv::imwrite(depthFile.str(), depthMat);
        }
    }

    stopCapturing();
}

/**
 * @brief Stop capturing frames from the connected Kinect device.
 */
void KinectHandler::stopCapturing() {
    if(_isRunning) {
        cvDestroyWindow(cvWindowTitleRgb);
        cvDestroyWindow(cvWindowTitleDepth);
    }

    if(_device) {
        _device->stopVideo();
        _device->stopDepth();
    }
}

/**
 * @brief Set the result output main directory.
 * @param outputDir Output directory.
 */
void KinectHandler::setOutputDir(const string &outputDir) {
    if(outputDir.size() == 0) {
        throw runtime_error("Output directory is empty.");
    }

    if(_outputDir.compare(outputDir) == 0) {
        return;
    }

    _outputDir = outputDir;

    const string separator = "/";

    if(_outputDir.rfind(separator) != _outputDir.size()) {
        _outputDir += separator;
    }
}

/**
 * @brief Associate RGB with depth image files. Takes in account that the closest
 * time difference between the files is used.
 * @param rgbDir Output directory that contains RGB data.
 * @param depthDir Output directory that contains depth data.
 * @param resultDir Base result directory to store the file associations.
 * @param maxDifferenceMs Maximum difference between timestamps, otherwise skip.
 */
void KinectHandler::associateFiles(const string &rgbDir, const string &depthDir, const string &resultDir, const bool &cleanUp, const int &maxDifferenceMs)
{
    string resDir = resultDir;

    if(!checkDir(resDir)) {
        throw runtime_error("Result direcory error: " + resultDir);
    }

    vector<string> rgbFiles = getFiles(rgbDir, "*." + _extension);
    vector<string> depthFiles = getFiles(depthDir, "*." + _extension);

    map<int, string> usedDepthFiles;


    ofstream resFile;
    resFile.open(resDir + _outputFile);
    resFile << "# Associated RGB and depth files with max difference of " << maxDifferenceMs << "ms\n";
    resFile << "# rgbFileName depthFileName\n";

    for(int i = 0; i < rgbFiles.size(); i++) {
        int64_t rgbTimestamp = stol(extractFileName(rgbFiles[i]).c_str(), nullptr);
        int64_t minDiff = numeric_limits<int64_t>::max();

        int foundCandidateIndex = -1;

        for(int j = 0, jLength = depthFiles.size(); j < jLength; j++) {
            int64_t depthTimestamp = stol(extractFileName(depthFiles[j]).c_str(), nullptr);

            int64_t diff = abs(depthTimestamp - rgbTimestamp);

            if(diff <= maxDifferenceMs) {
                if(diff < minDiff) {
                    minDiff = diff;
                    foundCandidateIndex = j;
                }
            }
        }

        if(foundCandidateIndex != -1) {
            resFile << rgbFiles[i] << " " << depthFiles[foundCandidateIndex] << "\n";

            usedDepthFiles[foundCandidateIndex] = depthFiles[foundCandidateIndex];
        }
    }
    resFile.close();

    if(cleanUp) {
        cout << "Cleaning up." << endl;
        for(int i = 0, iLength = depthFiles.size(); i < iLength; i++) {
            if(usedDepthFiles.find(i) == usedDepthFiles.end()) {
                if(remove((depthDir + "/" + depthFiles[i]).c_str()) == 0) {
                    cout << depthFiles[i] << endl;
                }
            }
        }
    }
}

/**
 * @brief Display results in an OpenCV output window.
 * @param rgb Captured RGB image.
 * @param depth Captured depth image.
 */
void KinectHandler::displayResults(const cv::Mat &rgb, const cv::Mat &depth) noexcept
{
    cv::Mat depthf(cv::Size(640,480), CV_8UC1);

    depth.convertTo(depthf, CV_8UC1, 255.0/2048.0);
    cv::applyColorMap(depthf, depthf, cv::COLORMAP_JET);

    cv::imshow(cvWindowTitleRgb, rgb);
    cv::imshow(cvWindowTitleDepth, depthf);
}

/**
 * @brief Create directory.
 * @param dirPath Directory path that should be created.
 */
void KinectHandler::createDirectory(const string &dirPath)
{
    if(dirPath.size() == 0) {
        throw runtime_error("Directory path is empty.");
    }

    const QDir dir(QString::fromStdString(dirPath));

    if(dir.exists()) {
        return;
    }

    QDir::root().mkpath(dir.absolutePath());
}

/**
 * @brief Checks directory path and corrects it if necessary.
 * @param dir Director path to check.
 * @return True/false if directory check is successful.
 */
bool KinectHandler::checkDir(string &dir) noexcept
{
    if(dir.size() == 0) {
        return false;
    }

    if(dir.rfind("/") != dir.size()) {
        dir += "/";
    }

    return true;
}

/**
 * @brief Extract the filename from the file.
 * @param file File path.
 * @return Base filename, e.g. 'file.tar.gz' will be 'file'.
 */
string KinectHandler::extractFileName(const string &file)
{
    if(file.size() ==  0) {
        throw runtime_error("Filename is empty.");
    }

    QFileInfo fileInfo(QString::fromStdString(file));
    string basename = fileInfo.baseName().toStdString();
    return basename;
}

/**
 * @brief KinectHandler::getFiles
 * @param dir Directory that should be listed.
 * @param filter File filter, e.g. "*.png"
 */
vector<string> KinectHandler::getFiles(const string &dir, const string &filter) noexcept
{
    const QStringList nameFilter(QString::fromStdString(filter));
    const QDir directory(QString::fromStdString(dir));
    const QStringList files = directory.entryList(nameFilter);

    vector<string> res;

    for(int i = 0, iLength = files.size(); i < iLength; i++) {
        res.push_back(files[i].toStdString());
    }

    sort(res.begin(), res.end());

    return res;
}

/**
 * @brief Set depth mode:
 * @param depthMode
 * FREENECT_DEPTH_11BIT        = 0, 11 bit depth information in one uint16_t/pixel
 * FREENECT_DEPTH_10BIT        = 1, 10 bit depth information in one uint16_t/pixel
 * FREENECT_DEPTH_11BIT_PACKED = 2, 11 bit packed depth information
 * FREENECT_DEPTH_10BIT_PACKED = 3, 10 bit packed depth information
 * FREENECT_DEPTH_REGISTERED   = 4, processed depth data in mm, aligned to 640x480 RGB
 * FREENECT_DEPTH_MM = 5, depth to each pixel in mm, but left unaligned to RGB image
*/
void KinectHandler::setDepthMode(const short &depthMode)
{
    if(depthMode < 0 || depthMode > 5) {
        return;
    }
    if(!_device) {
        return;
    }

    switch(depthMode) {
    case 0:
        _device->setDepthFormat(FREENECT_DEPTH_11BIT);
        cout << "depth format: FREENECT_DEPTH_11BIT" << endl;
        break;
    case 1:
        _device->setDepthFormat(FREENECT_DEPTH_10BIT);
        cout << "depth format: FREENECT_DEPTH_10BIT" << endl;
        break;
    case 2:
        _device->setDepthFormat(FREENECT_DEPTH_11BIT_PACKED);
        cout << "depth format: FREENECT_DEPTH_11BIT_PACKED" << endl;
        break;
    case 3:
        _device->setDepthFormat(FREENECT_DEPTH_10BIT_PACKED);
        cout << "depth format: FREENECT_DEPTH_10BIT_PACKED" << endl;
        break;
    case 4:
        _device->setDepthFormat(FREENECT_DEPTH_REGISTERED);
        cout << "depth format: FREENECT_DEPTH_REGISTERED" << endl;
        break;
    case 5:
        _device->setDepthFormat(FREENECT_DEPTH_MM);
        cout << "depth format: FREENECT_DEPTH_MM" << endl;
        break;
    }
}

/**
 * @brief KinectHandler::getDate
 * @return Curent date time, e.g. 2019-06-26_16:03:02
 */
string KinectHandler::getCurrentDateTime() noexcept
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}
