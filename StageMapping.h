#pragma once
#ifndef STAGE_MAPPING_H
#define STAGE_MAPPING_H

#include <opencv2/opencv.hpp>
#include "StereoCalibration.h"

// Converts a known physical stage movement (in mm) into pixel displacement
// for a given 3D point and camera intrinsics/distortion.
cv::Point2f stageToPixelShift(const cv::Point3f& point3D,
    float dx_mm, float dy_mm,
    const cv::Mat& K, const cv::Mat& D);

// Converts a measured pixel shift (in pixels) back into stage displacement (in mm)
// given focal length and estimated depth Z (meters).
cv::Point2f pixelToStageShift(const cv::Point2f& pixelShift,
    float depth_m,
    const cv::Mat& K);

#endif // STAGE_MAPPING_H
