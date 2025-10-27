#include "detectiontraverser.h"
#include "utils.h"
#include <vector>
#include "mainwindow.h"
#include <QString>
#include <QObject>
#include <QTextEdit>


DetectionTraverser::DetectionTraverser(XYZStage* xyzStage, QObject *parent)
    : QObject(parent), m_xyzStage(xyzStage), m_paused(false), m_aborted(false)
{
    log("appendLog working in detectiontraverser.", "INFO");
}

void DetectionTraverser::setTraversalData(const std::vector<cv::Rect>& path, const cv::Mat& transformMatrix)
{
    m_macroImgPath = path;
    m_transformMatrix = transformMatrix;
}

void DetectionTraverser::abortTraversal()
{
    QMutexLocker locker(&m_mutex);
    m_aborted = true;
    if (m_paused) {
        m_pauseCondition.wakeAll(); // Wake it up to allow it to exit
    }
}

void DetectionTraverser::userConfirmedAdjustment()
{
    QMutexLocker locker(&m_mutex);
    if (m_paused) {
        log("Resume state.", "INFO");
        m_paused = false;
        m_pauseCondition.wakeAll(); // Wake up the processing loop
    }
}

void DetectionTraverser::process()
{
    emit traversalStarted();
    m_aborted = false;

    // Convert all image coordinates to real coordinates at once
    std::vector<cv::Point2f> realCoordinates;
    for (const auto& box : m_macroImgPath) {
        cv::Point2f imageCenter(box.x + box.width / 2.0f, box.y + box.height / 2.0f);
        std::vector<cv::Point2f> imagePoints = { imageCenter };
        std::vector<cv::Point2f> transformedPoints;
        cv::transform(imagePoints, transformedPoints, m_transformMatrix);
        realCoordinates.push_back(transformedPoints[0]);
    }
    log(QString("Starting traversal of: %1 detected points.").arg(realCoordinates.size()), "INFO");
    //LOG_INFO("Starting traversal of " <<  << " detected points");

    for (size_t i = 0; i < realCoordinates.size(); ++i) {
        {
            QMutexLocker locker(&m_mutex);
            if (m_aborted) {
                log(QString("Trivarsal aborted"), "INFO");
                emit traversalFinished("Traversal aborted.");
                return;
            }
        }
        
        emit updateProgress(i + 1, realCoordinates.size());

        const cv::Point2f& targetPoint = realCoordinates[i];
        if (targetPoint.x < MIN_X || targetPoint.x > MAX_X ||
            targetPoint.y < MIN_Y || targetPoint.y > MAX_Y) {
            log(QString("Point %1 is out of bound. Skipping!").arg(i + 1), "INFO");
            //LOG_WARNING("Point " << (i + 1) << " out of bounds: (" << targetPoint.x << ", " << targetPoint.y << "). Skipping.");
            continue;  // Skip to next point
        }

        // Calculate deltas from the *current actual position*
        double deltaX = targetPoint.x - globle_vars.current_x;
        double deltaY = targetPoint.y - globle_vars.current_y;
        double deltaZ = 26650 - globle_vars.current_z; // Constant Z target

        log(QString("Point %1/%2: Moving...").arg(i + 1).arg(realCoordinates.size()), "INFO");

        // Use the new BLOCKING move function
        m_xyzStage->move_and_wait(deltaX, 0, 0);
        m_xyzStage->move_and_wait(0, deltaY, 0);
        m_xyzStage->move_and_wait(0, 0, deltaZ);
        log(QString("Arrived at point %1 ...Waiting for user adjustment.").arg(i + 1), "INFO");
        emit waitingForUserAdjustment();

        // Pause execution and wait for the user to click "Confirm"
        {
            QMutexLocker locker(&m_mutex);
            m_paused = true;
            m_pauseCondition.wait(&m_mutex);
            // After waking up, the loop will continue to the next point (or exit if aborted)
        }
    }

    emit traversalFinished("Traversal completed successfully.");
}