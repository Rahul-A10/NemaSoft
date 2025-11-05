#include "StereoCalibration.h"
#include <iostream>

StereoCalibration::StereoCalibration(cv::Size boardSize, float squareSize)
    : camCalibLeft_(boardSize, squareSize), camCalibRight_(boardSize, squareSize) {
}

bool StereoCalibration::runStereoCalibration(const std::vector<std::string>& leftImages,
    const std::vector<std::string>& rightImages,
    StereoResult& out,
    bool showCorners) {
    if (leftImages.size() != rightImages.size() || leftImages.empty()) {
        std::cerr << "Left and right image lists must be same non-zero length.\n";
        return false;
    }

    // Collect corners for left and right
    std::vector<std::vector<cv::Point2f>> leftPts, rightPts;
    cv::Size imgSizeLeft, imgSizeRight;
    if (!camCalibLeft_.collectChessboardCorners(leftImages, leftPts, imgSizeLeft, showCorners)) {
        std::cerr << "Failed to collect left corners.\n"; return false;
    }
    if (!camCalibRight_.collectChessboardCorners(rightImages, rightPts, imgSizeRight, showCorners)) {
        std::cerr << "Failed to collect right corners.\n"; return false;
    }
    if (imgSizeLeft != imgSizeRight) {
        std::cerr << "Left and right images have different sizes.\n";
        return false;
    }

    out.left = camCalibLeft_.calibrate(leftPts, imgSizeLeft);
    out.right = camCalibRight_.calibrate(rightPts, imgSizeRight);

    // prepare objectPoints (one per view)
    std::vector<std::vector<cv::Point3f>> objectPoints(leftPts.size(), camCalibLeft_.createObjectPoints());

    // stereoCalibrate
    cv::TermCriteria term(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5);
    int flags = cv::CALIB_FIX_INTRINSIC;
    double rms = cv::stereoCalibrate(objectPoints, leftPts, rightPts,
        out.left.cameraMatrix, out.left.distCoeffs,
        out.right.cameraMatrix, out.right.distCoeffs,
        imgSizeLeft, out.R, out.T, out.E, out.F,
        flags, term);
    out.stereoReprojError = rms;

    // Rectify and compute projection matrices and Q
    cv::stereoRectify(out.left.cameraMatrix, out.left.distCoeffs,
        out.right.cameraMatrix, out.right.distCoeffs,
        imgSizeLeft, out.R, out.T,
        out.R1, out.R2, out.P1, out.P2, out.Q,
        cv::CALIB_ZERO_DISPARITY, -1, imgSizeLeft);

    std::cout << "Stereo calibrate RMS error = " << rms << "\n";
    return true;
}

void StereoCalibration::save(const std::string& filename, const StereoResult& res) const {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "K1" << res.left.cameraMatrix << "D1" << res.left.distCoeffs;
    fs << "K2" << res.right.cameraMatrix << "D2" << res.right.distCoeffs;
    fs << "R" << res.R << "T" << res.T << "E" << res.E << "F" << res.F;
    fs << "R1" << res.R1 << "R2" << res.R2 << "P1" << res.P1 << "P2" << res.P2 << "Q" << res.Q;
    fs.release();
}

bool StereoCalibration::load(const std::string& filename, StereoResult& res) const {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["K1"] >> res.left.cameraMatrix; fs["D1"] >> res.left.distCoeffs;
    fs["K2"] >> res.right.cameraMatrix; fs["D2"] >> res.right.distCoeffs;
    fs["R"] >> res.R; fs["T"] >> res.T; fs["E"] >> res.E; fs["F"] >> res.F;
    fs["R1"] >> res.R1; fs["R2"] >> res.R2; fs["P1"] >> res.P1; fs["P2"] >> res.P2; fs["Q"] >> res.Q;
    fs.release();
    return true;
}

cv::Point3f StereoCalibration::triangulate(const cv::Point2f& pl, const cv::Point2f& pr,
    const StereoResult& res) {
    // Use projection matrices P1 and P2 (from stereoRectify) to triangulate
    cv::Mat pts4d;
    cv::Mat plh(2, 1, CV_64F), prh(2, 1, CV_64F);
    plh.at<double>(0, 0) = pl.x; plh.at<double>(1, 0) = pl.y;
    prh.at<double>(0, 0) = pr.x; prh.at<double>(1, 0) = pr.y;

    cv::triangulatePoints(res.P1, res.P2, plh, prh, pts4d); // 4xN

    cv::Mat p = pts4d.col(0);
    double w = p.at<double>(3, 0);
    if (fabs(w) < 1e-10) w = 1e-10;
    cv::Point3f pt((float)(p.at<double>(0, 0) / w),
        (float)(p.at<double>(1, 0) / w),
        (float)(p.at<double>(2, 0) / w));
    return pt;
}
