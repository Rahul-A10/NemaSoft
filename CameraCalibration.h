#pragma once
#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct CalibResult {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double reprojectionError;
};

class CameraCalibration {
public:
    CameraCalibration(cv::Size boardSize, float squareSize);

    // Find chessboard corners in a list of image file paths
    bool collectChessboardCorners(const std::vector<std::string>& imageFiles,
        std::vector<std::vector<cv::Point2f>>& imagePoints,
        cv::Size& imageSize,
        bool show = false);

    // Run calibrateCamera given found imagePoints
    CalibResult calibrate(const std::vector<std::vector<cv::Point2f>>& imagePoints,
        const cv::Size& imageSize);

    // Utility: generate object points (same for each image)
    std::vector<cv::Point3f> createObjectPoints() const;

private:
    cv::Size boardSize_; // number of inner corners (cols, rows)
    float squareSize_;
};

#endif // CAMERA_CALIBRATION_H
