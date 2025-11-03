// PointMatcher.h
#ifndef POINTMATCHER_H
#define POINTMATCHER_H

#include <QObject>
#include <QPoint>
#include <QPointF>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "src/utils.h"

class PointMatcher : public QObject {
    Q_OBJECT

public:
    explicit PointMatcher(QObject* parent = nullptr);
    ~PointMatcher();

    // Main function: Find corresponding point in the other camera
    bool findCorrespondingPoint(const cv::Mat& sourceFrame,
        const cv::Mat& targetFrame,
        const QPointF& sourcePoint,
        QPointF& correspondingPoint);

    // Configuration
    void setSearchRadius(int radius);           // Radius around clicked point to search for keypoints
    void setMatchRatioThreshold(float ratio);   // Lowe's ratio test threshold (default 0.7)
    void setMinMatchQuality(float quality);     // Minimum match quality (0-1)

    // Get last match confidence (0-1, higher is better)
    float getLastMatchConfidence() const { return m_lastMatchConfidence; }

signals:
    void matchFound(QPointF targetPoint, float confidence);
    void matchFailed(const QString& reason);
    void logMessage(const QString& message, const QString& level);

private:
    // ORB detector and matcher (instead of SIFT)
    cv::Ptr<cv::ORB> m_orbDetector;
    cv::Ptr<cv::DescriptorMatcher> m_matcher;

    // Configuration parameters
    int m_searchRadius;           // Pixel radius to search for keypoints near click
    float m_matchRatioThreshold;  // Lowe's ratio test threshold
    float m_minMatchQuality;      // Minimum acceptable match quality

    // State
    float m_lastMatchConfidence;

    // Helper methods
    int findClosestKeypoint(const std::vector<cv::KeyPoint>& keypoints,
        const QPointF& point);

    float calculateMatchConfidence(const std::vector<cv::DMatch>& matches,
        int selectedMatchIdx);

    bool verifyMatch(const cv::Mat& frame1, const cv::Mat& frame2,
        const QPointF& point1, const QPointF& point2);

    void drawDebugVisualization(const cv::Mat& frame1, const cv::Mat& frame2,
        const QPointF& point1, const QPointF& point2,
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::KeyPoint>& kp2,
        const std::vector<cv::DMatch>& matches);
};

#endif // POINTMATCHER_H