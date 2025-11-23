#include "StageMapping.h"
#include <iostream>

StageMapping::StageMapping()
    : calibrated_frame1_(false), calibrated_frame2_(false) {
}

// --------------------------------------------------------------------------
void StageMapping::setCalibrationData(CameraID camID, const std::vector<PointMapping>& points) {
    if (points.size() < 4) {
        std::cerr << "Need at least 4 calibration points for camera "
            << (camID == CameraID::FRAME1 ? "FRAME1" : "FRAME2") << "!" << std::endl;
        return;
    }

    // Reference (origin)
    double u0 = points[0].u;
    double v0 = points[0].v;
    double X0 = points[0].X;
    double Y0 = points[0].Y;

    int N = static_cast<int>(points.size());
    Eigen::MatrixXd U(N, 2);
    Eigen::VectorXd dX(N);
    Eigen::VectorXd dY(N);

    for (int i = 0; i < N; ++i) {
        U(i, 0) = points[i].u - u0;
        U(i, 1) = points[i].v - v0;
        dX(i) = points[i].X - X0;
        dY(i) = points[i].Y - Y0;
    }

    Eigen::Vector2d A_x = (U.transpose() * U).ldlt().solve(U.transpose() * dX);
    Eigen::Vector2d A_y = (U.transpose() * U).ldlt().solve(U.transpose() * dY);
    Eigen::Matrix2d A;
    A.row(0) = A_x.transpose();
    A.row(1) = A_y.transpose();

    if (camID == CameraID::FRAME1) {
        A_frame1_ = A;
        calibrated_frame1_ = true;
    }
    else {
        A_frame2_ = A;
        calibrated_frame2_ = true;
    }
}

// --------------------------------------------------------------------------
Eigen::Vector2d StageMapping::computeStageDelta(CameraID camID, double u, double v, double uc, double vc) const {
    

    if (camID == CameraID::FRAME1) {
        double du = u - uc;
        double dv = v - vc;
        Eigen::Vector2d pixelDelta(du,dv);
        if (!calibrated_frame1_) {
            std::cerr << "FRAME1 not calibrated!" << std::endl;
            return Eigen::Vector2d::Zero();
        }
        return A_frame1_ * pixelDelta;
    }
    else {
        double du = u - uc;
        double dv = v - vc;
        Eigen::Vector2d pixelDelta(du, dv);
        if (!calibrated_frame2_) {
            
            std::cerr << "FRAME2 not calibrated!" << std::endl;
            return Eigen::Vector2d::Zero();
        }
        return A_frame2_ * pixelDelta;
    }
}

// --------------------------------------------------------------------------
void StageMapping::printMatrix(CameraID camID) const {
    if (camID == CameraID::FRAME1) {
        if (!calibrated_frame1_) {
            std::cerr << "FRAME1 not calibrated yet." << std::endl;
            return;
        }
        std::cout << "Stage-to-image mapping matrix A (FRAME1):\n" << A_frame1_ << std::endl;
    }
    else {
        if (!calibrated_frame2_) {
            std::cerr << "FRAME2 not calibrated yet." << std::endl;
            return;
        }
        std::cout << "Stage-to-image mapping matrix A (FRAME2):\n" << A_frame2_ << std::endl;
    }
}
