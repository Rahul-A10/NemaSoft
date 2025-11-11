#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>

struct PointMapping {
    double u;
    double v;
    double X;
    double Y;
};

// Enum to choose camera
enum class CameraID {
    FRAME1 = 0,
    FRAME2 = 1
};

class StageMapping {
public:
    StageMapping();

    // Calibration for each frame
    void setCalibrationData(CameraID camID, const std::vector<PointMapping>& points);

    // Compute stage delta for a given image coordinate (u,v)
    Eigen::Vector2d computeStageDelta(CameraID camID, double u, double v, double uc, double vc) const;

    // Print matrix for each frame
    void printMatrix(CameraID camID) const;

private:
    Eigen::Matrix2d A_frame1_;
    Eigen::Matrix2d A_frame2_;
    bool calibrated_frame1_;
    bool calibrated_frame2_;
};
