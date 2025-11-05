#include "StageMapping.h"
#include <iostream>

// --- Forward mapping: stage movement → pixel movement ---
cv::Point2f stageToPixelShift(const cv::Point3f& point3D,
    float dx_mm, float dy_mm,
    const cv::Mat& K, const cv::Mat& D)
{
    // Convert mm to meters
    cv::Point3f P1 = point3D;
    cv::Point3f P2 = { point3D.x + dx_mm / 1000.0f,
                       point3D.y + dy_mm / 1000.0f,
                       point3D.z };

    std::vector<cv::Point3f> pts3d = { P1, P2 };
    std::vector<cv::Point2f> pts2d;
    cv::projectPoints(pts3d,
        cv::Mat::zeros(3, 1, CV_64F),  // no rotation
        cv::Mat::zeros(3, 1, CV_64F),  // no translation
        K, D, pts2d);

    return pts2d[1] - pts2d[0];
}

// --- Inverse mapping: pixel movement → stage movement ---
cv::Point2f pixelToStageShift(const cv::Point2f& pixelShift,
    float depth_m,
    const cv::Mat& K)
{
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);

    // ΔX = Δu * Z / fx , ΔY = Δv * Z / fy
    float dx_m = pixelShift.x * depth_m / fx;
    float dy_m = pixelShift.y * depth_m / fy;

    return cv::Point2f(dx_m * 1000.0f, dy_m * 1000.0f); // meters → mm
}
