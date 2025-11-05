#include "CameraCalibration.h"
#include <iostream>

CameraCalibration::CameraCalibration(cv::Size boardSize, float squareSize)
    : boardSize_(boardSize), squareSize_(squareSize) {
}

std::vector<cv::Point3f> CameraCalibration::createObjectPoints() const {
    std::vector<cv::Point3f> obj;
    for (int r = 0; r < boardSize_.height; ++r) {
        for (int c = 0; c < boardSize_.width; ++c) {
            obj.push_back(cv::Point3f(c * squareSize_, r * squareSize_, 0));
        }
    }
    return obj;
}

bool CameraCalibration::collectChessboardCorners(const std::vector<std::string>& imageFiles,
    std::vector<std::vector<cv::Point2f>>& imagePoints,
    cv::Size& imageSize,
    bool show) {
    imagePoints.clear();
    imageSize = cv::Size(0, 0);
    for (const auto& file : imageFiles) {
        cv::Mat gray;
        cv::Mat img = cv::imread(file);
        if (img.empty()) {
            std::cerr << "Failed to read image: " << file << "\n";
            return false;
        }
        if (imageSize.width == 0) imageSize = img.size();

        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, boardSize_, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            imagePoints.push_back(corners);
            if (show) {
                cv::drawChessboardCorners(img, boardSize_, corners, found);
                cv::imshow("corners", img);
                cv::waitKey(200);
            }
        }
        else {
            std::cerr << "Chessboard not found in " << file << " (skipping).\n";
        }
    }
    if (show) cv::destroyAllWindows();
    return !imagePoints.empty();
}

CalibResult CameraCalibration::calibrate(const std::vector<std::vector<cv::Point2f>>& imagePoints,
    const cv::Size& imageSize) {
    CalibResult res;
    std::vector<std::vector<cv::Point3f>> objectPoints(imagePoints.size(), createObjectPoints());

    res.cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    res.distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
        res.cameraMatrix, res.distCoeffs,
        res.rvecs, res.tvecs,
        cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
    res.reprojectionError = rms;
    return res;
}
