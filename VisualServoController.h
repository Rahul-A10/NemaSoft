#pragma once

// VisualServoController.h
#ifndef VISUALSERVOCONTROLLER_H
#define VISUALSERVOCONTROLLER_H

#include <QObject>
#include <QPoint>
#include <functional>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "src/XYZStage.h"

class VisualServoController : public QObject {
    Q_OBJECT

public:
    explicit VisualServoController(XYZStage* stage, QObject* parent = nullptr);

    // Main servoing function: Move stage until target points are centered in both views
    bool servoToTarget(const QPoint& targetCam1, const QPoint& targetCam2,
        std::function<cv::Mat()> getCam1Frame,
        std::function<cv::Mat()> getCam2Frame);

    // Configuration
    void setStepSize(double xy_step, double z_step);
    void setVelocity(double vx, double vy, double vz);
    void setConvergenceThreshold(double pixels);
    void setMaxIterations(int max_iter);
    void setCenterTarget(bool center); // If true, servo to center; if false, servo to clicked point

    // Get current frame centers
    QPoint getFrameCenter(const cv::Mat& frame);

signals:
    void servoIterationComplete(int iteration, double error1, double error2);
    void servoComplete(bool success, int iterations);
    void errorOccurred(const QString& error);

private:
    XYZStage* m_stage;

    // Servo parameters
    double m_stepXY;              // Movement step size in XY (mm)
    double m_stepZ;               // Movement step size in Z (mm)
    double m_velocityX;
    double m_velocityY;
    double m_velocityZ;
    double m_convergenceThreshold; // Pixel threshold for convergence
    int m_maxIterations;
    bool m_centerTarget;           // True = move to center, False = stay at clicked point

    // Helper methods
    double calculateError(const QPoint& current, const QPoint& target);
    cv::Point2f computeMovementDirection(const QPoint& currentCam1, const QPoint& targetCam1,
        const QPoint& currentCam2, const QPoint& targetCam2,
        const cv::Mat& frame1, const cv::Mat& frame2);
    QPoint trackPoint(const cv::Mat& prevFrame, const cv::Mat& currFrame, const QPoint& point);
};

#endif // VISUALSERVOCONTROLLER_H