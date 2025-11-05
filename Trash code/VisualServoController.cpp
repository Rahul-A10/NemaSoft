// VisualServoController.cpp
#include "VisualServoController.h"
#include "logger.h"
#include <cmath>

VisualServoController::VisualServoController(XYZStage* stage, QObject* parent)
    : QObject(parent)
    , m_stage(stage)
    , m_stepXY(0.5)                    // 0.5mm steps in XY
    , m_stepZ(0.2)                     // 0.2mm steps in Z
    , m_velocityX(1000)
    , m_velocityY(1000)
    , m_velocityZ(500)
    , m_convergenceThreshold(5.0)     // 5 pixels
    , m_maxIterations(50)
    , m_centerTarget(true)             // Default: move to center
    , m_pixelsPerMM_X(10.0)    // DEFAULT: 10 pixels = 1mm (CALIBRATE THIS!)
    , m_pixelsPerMM_Y(10.0)    // DEFAULT: 10 pixels = 1mm (CALIBRATE THIS!)
    , m_invertX(false)          // Set to true if X moves opposite
    , m_invertY(false)          // Set to true if Y moves opposite
{
    Logger::instance().log("VisualServoController initialized", "INFO");
}



void VisualServoController::setImageToStageMapping(double pixelsPerMM_X, double pixelsPerMM_Y,
    bool invertX, bool invertY) {
    m_pixelsPerMM_X = pixelsPerMM_X;
    m_pixelsPerMM_Y = pixelsPerMM_Y;
    m_invertX = invertX;
    m_invertY = invertY;

    Logger::instance().log(QString("Image-to-stage mapping set: %1 px/mm (X), %2 px/mm (Y), invert X=%3, Y=%4")
        .arg(pixelsPerMM_X).arg(pixelsPerMM_Y).arg(invertX).arg(invertY), "INFO");
}



void VisualServoController::setStepSize(double xy_step, double z_step) {
    m_stepXY = xy_step;
    m_stepZ = z_step;
    Logger::instance().log(QString("Step size set: XY=%1mm, Z=%2mm")
        .arg(xy_step).arg(z_step), "INFO");
}

void VisualServoController::setVelocity(double vx, double vy, double vz) {
    m_velocityX = vx;
    m_velocityY = vy;
    m_velocityZ = vz;
}

void VisualServoController::setConvergenceThreshold(double pixels) {
    m_convergenceThreshold = pixels;
    Logger::instance().log(QString("Convergence threshold set: %1 pixels").arg(pixels), "INFO");
}

void VisualServoController::setMaxIterations(int max_iter) {
    m_maxIterations = max_iter;
}

void VisualServoController::setCenterTarget(bool center) {
    m_centerTarget = center;
    Logger::instance().log(QString("Target mode: %1")
        .arg(center ? "Center of frame" : "Clicked position"), "INFO");
}

QPoint VisualServoController::getFrameCenter(const cv::Mat& frame) {
    return QPoint(frame.cols / 2, frame.rows / 2);
}

double VisualServoController::calculateError(const QPoint& current, const QPoint& target) {
    double dx = current.x() - target.x();
    double dy = current.y() - target.y();
    return sqrt(dx * dx + dy * dy);
}

QPoint VisualServoController::trackPoint(const cv::Mat& prevFrame,
    const cv::Mat& currFrame,
    const QPoint& point) {
    // Use optical flow to track the point
    std::vector<cv::Point2f> prevPoints = { cv::Point2f(point.x(), point.y()) };
    std::vector<cv::Point2f> currPoints;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::Mat prevGray, currGray;

    // Check if frames are already grayscale
    if (prevFrame.channels() == 3) {
        cv::cvtColor(prevFrame, prevGray, cv::COLOR_BGR2GRAY);
    }
    else {
        prevGray = prevFrame;
    }

    if (currFrame.channels() == 3) {
        cv::cvtColor(currFrame, currGray, cv::COLOR_BGR2GRAY);
    }
    else {
        currGray = currFrame;
    }

    cv::calcOpticalFlowPyrLK(prevGray, currGray, prevPoints, currPoints,
        status, err, cv::Size(21, 21), 3);

    if (status[0] == 1) {
        return QPoint(static_cast<int>(currPoints[0].x),
            static_cast<int>(currPoints[0].y));
    }

    Logger::instance().log("Optical flow tracking failed, returning original point", "WARNING");
    return point; // Return original if tracking failed
}

cv::Point2f VisualServoController::computeMovementDirection(
    const QPoint& currentCam1, const QPoint& targetCam1,
    const QPoint& currentCam2, const QPoint& targetCam2,
    const cv::Mat& frame1, const cv::Mat& frame2) 
{
    // Calculate X and Y errors for both cameras
    double error1_x = currentCam1.x() - targetCam1.x();
    double error2_x = currentCam2.x() - targetCam2.x();
    double error1_y = currentCam1.y() - targetCam1.y();
    double error2_y = currentCam2.y() - targetCam2.y();

    double pixelError_X = (error1_x + error2_x) / 2.0;  // Average X error in pixels
    double pixelError_Y = (error1_y + error2_y) / 2.0;  // Average Y error in pixels

    // Convert pixel error to mm
    double errorMM_X = pixelError_X / m_pixelsPerMM_X;
    double errorMM_Y = pixelError_Y / m_pixelsPerMM_Y;

    // Apply step size (don't move full error, just a fraction)
    double dx = errorMM_X * 0.5;  // Move 50% of the error
    double dy = errorMM_Y * 0.5;  // Move 50% of the error

    // Apply direction inversions if needed
    if (m_invertX) dx = -dx;
    if (m_invertY) dy = -dy;

    // Clamp to max step size
    dx = std::max(-m_stepXY, std::min(m_stepXY, dx));
    dy = std::max(-m_stepXY, std::min(m_stepXY, dy));

    double dz = 0.0;  // Z is always 0

    Logger::instance().log(QString("Moving stage: dx=%1mm, dy=%2mm (from errors: %3px, %4px)")
        .arg(dx, 0, 'f', 3)
        .arg(dy, 0, 'f', 3)
        .arg(pixelError_X, 0, 'f', 1)
        .arg(pixelError_Y, 0, 'f', 1), "INFO");

    return cv::Point2f(dx, dy);
}

bool VisualServoController::servoToTarget(const QPoint& initialTargetCam1,
    const QPoint& initialTargetCam2,
    std::function<cv::Mat()> getCam1Frame,
    std::function<cv::Mat()> getCam2Frame) {

    Logger::instance().log("Starting visual servoing loop (XY only, Z locked)", "INFO");

    cv::Mat frame1 = getCam1Frame();
    cv::Mat frame2 = getCam2Frame();

    if (frame1.empty() || frame2.empty()) {
        Logger::instance().log("Invalid frames received", "CRITICAL");
        emit errorOccurred("Cannot get camera frames");
        return false;
    }

    // Determine targets based on mode
    QPoint targetCam1 = m_centerTarget ? getFrameCenter(frame1) : initialTargetCam1;
    QPoint targetCam2 = m_centerTarget ? getFrameCenter(frame2) : initialTargetCam2;

    Logger::instance().log(QString("Servo targets - Cam1: (%1, %2), Cam2: (%3, %4)")
        .arg(targetCam1.x()).arg(targetCam1.y())
        .arg(targetCam2.x()).arg(targetCam2.y()), "INFO");

    // Track current positions of the points (start at initial clicked positions)
    QPoint currentCam1 = initialTargetCam1;
    QPoint currentCam2 = initialTargetCam2;

    cv::Mat prevFrame1 = frame1.clone();
    cv::Mat prevFrame2 = frame2.clone();

    // Main servoing loop
    for (int iteration = 0; iteration < m_maxIterations; iteration++) {
        // Get fresh frames from cameras
        frame1 = getCam1Frame();
        frame2 = getCam2Frame();

        if (frame1.empty() || frame2.empty()) {
            Logger::instance().log("Lost camera frames during servoing", "CRITICAL");
            emit servoComplete(false, iteration);
            return false;
        }

        // Track points in new frames using optical flow
        currentCam1 = trackPoint(prevFrame1, frame1, currentCam1);
        currentCam2 = trackPoint(prevFrame2, frame2, currentCam2);

        // Calculate errors (distance from target in pixels)
        double error1 = calculateError(currentCam1, targetCam1);
        double error2 = calculateError(currentCam2, targetCam2);
        double totalError = (error1 + error2) / 2.0;

        Logger::instance().log(QString("Iteration %1: Error Cam1=%2px, Cam2=%3px, Avg=%4px")
            .arg(iteration)
            .arg(error1, 0, 'f', 2)
            .arg(error2, 0, 'f', 2)
            .arg(totalError, 0, 'f', 2), "INFO");

        emit servoIterationComplete(iteration, error1, error2);

        // Check convergence
        if (totalError < m_convergenceThreshold) {
            Logger::instance().log(QString("Converged successfully in %1 iterations!")
                .arg(iteration), "INFO");
            emit servoComplete(true, iteration);
            return true;
        }

        // Compute movement direction from both camera errors
        cv::Point2f direction = computeMovementDirection(
            currentCam1, targetCam1, currentCam2, targetCam2, frame1, frame2);

        // Map image space movement to stage coordinates
        // Simple mapping: image X → stage X, image Y → stage Y
        double dx = direction.x * m_stepXY;
        double dy = direction.y * m_stepXY;
        double dz = 0.0;  // Z is always 0 - no Z movement

        Logger::instance().log(QString("Moving stage (XY only): dx=%1mm, dy=%2mm, dz=0mm")
            .arg(dx, 0, 'f', 3)
            .arg(dy, 0, 'f', 3), "INFO");

        // Move the stage in XY only, Z is locked at 0
        m_stage->move_and_wait(dx * 1000, dy * 1000, 0.0, m_velocityX, m_velocityY, m_velocityZ);

        // Store current frames for next iteration's optical flow
        prevFrame1 = frame1.clone();
        prevFrame2 = frame2.clone();
    }

    // Failed to converge within max iterations
    Logger::instance().log(QString("Failed to converge after %1 iterations")
        .arg(m_maxIterations), "WARNING");
    emit servoComplete(false, m_maxIterations);
    return false;
}