#ifndef STAGE_CALIBRATION_H
#define STAGE_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <QPointF>
#include <cmath>

class StageCalibration {
public:
    struct CalibrationResult {
        double scale_x;      // Stage units per pixel in X
        double scale_y;      // Stage units per pixel in Y
        int invert_x;       // 0 or 1 (1 if axis is inverted)
        int invert_y;       // 0 or 1 (1 if axis is inverted)
        bool valid;
    };

    // Calculate calibration from known points
    static CalibrationResult calibrate(
        cv::Point2f img_before,     // Image point before movement
        cv::Point2f img_after,      // Same point after movement
        double stage_move_x,        // Known stage movement in X
        double stage_move_y)        // Known stage movement in Y
    {
        CalibrationResult result;

        // Calculate pixel displacement
        double pixel_delta_x = img_after.x - img_before.x;
        double pixel_delta_y = img_after.y - img_before.y;

        // Prevent division by zero
        if (std::abs(pixel_delta_x) < 0.01 || std::abs(pixel_delta_y) < 0.01) {
            result.valid = false;
            return result;
        }

        // Calculate scaling factors (absolute value)
        result.scale_x = std::abs(stage_move_x / pixel_delta_x);
        result.scale_y = std::abs(stage_move_y / pixel_delta_y);

        // Determine inversion
        // If stage moved positive but pixels moved negative (or vice versa), axis is inverted
        result.invert_x = ((stage_move_x > 0) != (pixel_delta_x > 0)) ? 1 : 0;
        result.invert_y = ((stage_move_y > 0) != (pixel_delta_y > 0)) ? 1 : 0;

        result.valid = true;
        return result;
    }

    // Calculate stage movement to center a point
    static QPointF calculateMovement(
        const CalibrationResult& calib,
        cv::Point2f current_point,      // Current position in image
        cv::Point2f target_point)       // Target position (usually image center)
    {
        if (!calib.valid) {
            return QPointF(0, 0);
        }

        // Calculate pixel displacement needed
        double pixel_delta_x = target_point.x - current_point.x;
        double pixel_delta_y = target_point.y - current_point.y;

        // Apply transformation with scaling and inversion
        double stage_delta_x = pixel_delta_x * calib.scale_x * std::pow(-1, calib.invert_x);
        double stage_delta_y = pixel_delta_y * calib.scale_y * std::pow(-1, calib.invert_y);

        return QPointF(stage_delta_x, stage_delta_y);
    }

    // Simplified function for dual camera centering
    static QPointF centerPointDualCamera(
        const CalibrationResult& cam1_calib,
        const CalibrationResult& cam2_calib,
        cv::Point2f cam1_point,          // Point to center in camera 1
        cv::Point2f cam2_point,          // Corresponding point in camera 2
        cv::Size cam1_size,              // Camera 1 frame size
        cv::Size cam2_size)              // Camera 2 frame size
    {
        // Calculate center points
        cv::Point2f cam1_center(cam1_size.width / 2.0f, cam1_size.height / 2.0f);
        cv::Point2f cam2_center(cam2_size.width / 2.0f, cam2_size.height / 2.0f);

        // Calculate movement for each camera
        QPointF move1 = calculateMovement(cam1_calib, cam1_point, cam1_center);
        QPointF move2 = calculateMovement(cam2_calib, cam2_point, cam2_center);

        // Average the movements (they should be similar if calibration is good)
        return QPointF((move1.x() + move2.x()) / 2.0,
            (move1.y() + move2.y()) / 2.0);
    }
};

#endif // STAGE_CALIBRATION_H