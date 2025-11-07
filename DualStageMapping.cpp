#include "DualStageMapping.h"

// ----------------------------
// Single Camera Calibration
// ----------------------------
bool DualStageMapping::calibrateCamera(CameraConfig& config) {
    // Define image plane corner points (in pixels)
    std::vector<cv::Point2f> image_points = {
        cv::Point2f(0, 0),
        cv::Point2f(config.frame_width, 0),
        cv::Point2f(config.frame_width, config.frame_height),
        cv::Point2f(0, config.frame_height)
    };

    // Corresponding real-world stage coordinates
    std::vector<cv::Point2f> world_points = {
        cv::Point2f(config.origin_x, config.origin_y),
        cv::Point2f(config.origin_x + config.fov_width, config.origin_y),
        cv::Point2f(config.origin_x + config.fov_width, config.origin_y + config.fov_height),
        cv::Point2f(config.origin_x, config.origin_y + config.fov_height)
    };

    // Compute Homography (image -> world)
    config.homography = cv::findHomography(image_points, world_points);

    // Compute inverse (world -> image)
    config.homography_inv = cv::findHomography(world_points, image_points);

    config.is_calibrated = !config.homography.empty() && !config.homography_inv.empty();
    return config.is_calibrated;
}

// ----------------------------
// Image → Stage
// ----------------------------
QPointF DualStageMapping::imageToStage(const CameraConfig& config, const QPointF& imagePoint) {
    if (!config.is_calibrated) {
        qWarning() << "Camera not calibrated!";
        return QPointF(0, 0);
    }

    std::vector<cv::Point2f> src = { cv::Point2f(imagePoint.x(), imagePoint.y()) };
    std::vector<cv::Point2f> dst;

    cv::perspectiveTransform(src, dst, config.homography);

    if (!dst.empty())
        return QPointF(dst[0].x, dst[0].y);
    else
        return QPointF(0, 0);
}

// ----------------------------
// Stage → Image
// ----------------------------
QPointF DualStageMapping::stageToImage(const CameraConfig& config, const QPointF& stagePoint) {
    if (!config.is_calibrated) {
        qWarning() << "Camera not calibrated!";
        return QPointF(0, 0);
    }

    std::vector<cv::Point2f> src = { cv::Point2f(stagePoint.x(), stagePoint.y()) };
    std::vector<cv::Point2f> dst;

    cv::perspectiveTransform(src, dst, config.homography_inv);

    if (!dst.empty())
        return QPointF(dst[0].x, dst[0].y);
    else
        return QPointF(0, 0);
}

// ----------------------------
// Calculate centering movement
// ----------------------------
QPointF DualStageMapping::calculateCenteringMovement(
    const CameraConfig& config,
    const QPointF& currentImagePoint) {

    if (!config.is_calibrated)
        return QPointF(0, 0);

    // Image center in pixels
    QPointF imageCenter(config.frame_width / 2.0, config.frame_height / 2.0);

    // Convert both points to stage coordinates
    QPointF currentStage = imageToStage(config, currentImagePoint);
    QPointF centerStage = imageToStage(config, imageCenter);

    // Movement required in stage coordinates
    double deltaX = centerStage.x() - currentStage.x();
    double deltaY = centerStage.y() - currentStage.y();

    return QPointF(deltaX, deltaY);
}

// ----------------------------
// Dual Camera Independent Control
// ----------------------------
QPointF DualStageMapping::calculateMovementForLeftCamera(
    const CameraConfig& leftConfig,
    const QPointF& leftImagePoint) {

    return calculateCenteringMovement(leftConfig, leftImagePoint);
}

QPointF DualStageMapping::calculateMovementForRightCamera(
    const CameraConfig& rightConfig,
    const QPointF& rightImagePoint) {

    return calculateCenteringMovement(rightConfig, rightImagePoint);
}
