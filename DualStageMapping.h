#ifndef DUALSTAGEMAPPING_H
#define DUALSTAGEMAPPING_H

#include <opencv2/opencv.hpp>
#include <QPointF>
#include <QDebug>

class DualStageMapping {
public:
    struct CameraConfig {
        // Frame dimensions (in pixels)
        int frame_width ;
        int frame_height;
        // Real-world coordinates (stage units, e.g., µm or mm)
        double origin_x;
        double origin_y;
        double fov_width;
        double fov_height;
        // Computed matrices
        cv::Mat homography;
        cv::Mat homography_inv;
        bool is_calibrated = false;
    };

    // Initialize calibration for one camera
    static bool calibrateCamera(CameraConfig& config);
    // Transform image point → stage coordinates
    static QPointF imageToStage(const CameraConfig& config, const QPointF& imagePoint);
    // Transform stage coordinates → image point
    static QPointF stageToImage(const CameraConfig& config, const QPointF& stagePoint);

    // Calculate delta movement needed to center a point for this camera
    static QPointF calculateCenteringMovement(
        const CameraConfig& config,
        const QPointF& currentImagePoint);

    // ---- Dual Camera Specific ----
    // Independent control: move object based on one camera’s data
    static QPointF calculateMovementForLeftCamera(
        const CameraConfig& leftConfig,
        const QPointF& leftImagePoint);

    static QPointF calculateMovementForRightCamera(
        const CameraConfig& rightConfig,
        const QPointF& rightImagePoint);
};

#endif // DUALSTAGEMAPPING_H
