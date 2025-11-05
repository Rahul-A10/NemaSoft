#pragma once
#ifndef STEREO_CALIBRATION_H
#define STEREO_CALIBRATION_H

#include "CameraCalibration.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct StereoResult {
    CalibResult left, right;
    cv::Mat R, T, E, F;
    cv::Mat R1, R2, P1, P2, Q; // rectification
    double stereoReprojError;
};

class StereoCalibration {
public:
    StereoCalibration(cv::Size boardSize, float squareSize);

    // Provide paired image lists (leftImages and rightImages must be same order/length)
    bool runStereoCalibration(const std::vector<std::string>& leftImages,
        const std::vector<std::string>& rightImages,
        StereoResult& out,
        bool showCorners = false);

    // Save/Load YAML
    void save(const std::string& filename, const StereoResult& res) const;
    bool load(const std::string& filename, StereoResult& res) const;

    // Triangulate two pixel points (in left & right image) -> 3D point in left-camera frame
    // Both points are in pixels (cv::Point2f)
    static cv::Point3f triangulate(const cv::Point2f& pl, const cv::Point2f& pr,
        const StereoResult& res);

private:
    CameraCalibration camCalibLeft_, camCalibRight_;
};

#endif // STEREO_CALIBRATION_H
