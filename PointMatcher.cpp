// PointMatcher.cpp
#include "PointMatcher.h"
#include "logger.h"
#include <cmath>
#include <algorithm>

PointMatcher::PointMatcher(QObject* parent)
    : QObject(parent)
    , m_searchRadius(50)           // 50 pixel search radius
    , m_matchRatioThreshold(0.75f) // Slightly relaxed for stereo matching
    , m_minMatchQuality(0.6f)      // Minimum 60% confidence
    , m_lastMatchConfidence(0.0f)
{
    // Initialize ORB detector with parameters optimized for matching
    // ORB is faster than SIFT and doesn't require opencv_contrib
    m_orbDetector = cv::ORB::create(
        2000,     // nfeatures - max number of features to detect
        1.2f,     // scaleFactor
        8,        // nlevels - pyramid levels
        31,       // edgeThreshold
        0,        // firstLevel
        2,        // WTA_K
        cv::ORB::HARRIS_SCORE,  // scoreType
        31,       // patchSize
        20        // fastThreshold
    );

    // Use BFMatcher for ORB (binary descriptors)
    m_matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true); // crossCheck enabled

    Logger::instance().log("PointMatcher initialized with ORB detector", "INFO");
}

PointMatcher::~PointMatcher() {
}

void PointMatcher::setSearchRadius(int radius) {
    m_searchRadius = radius;
    Logger::instance().log(QString("Search radius set to %1 pixels").arg(radius), "INFO");
}

void PointMatcher::setMatchRatioThreshold(float ratio) {
    m_matchRatioThreshold = ratio;
    Logger::instance().log(QString("Match ratio threshold set to %1").arg(ratio), "INFO");
}

void PointMatcher::setMinMatchQuality(float quality) {
    m_minMatchQuality = quality;
    Logger::instance().log(QString("Minimum match quality set to %1").arg(quality), "INFO");
}

int PointMatcher::findClosestKeypoint(const std::vector<cv::KeyPoint>& keypoints,
    const QPointF& point) {
    if (keypoints.empty()) {
        return -1;
    }

    int closestIdx = -1;
    float minDist = std::numeric_limits<float>::max();

    for (size_t i = 0; i < keypoints.size(); i++) {
        float dx = keypoints[i].pt.x - point.x();
        float dy = keypoints[i].pt.y - point.y();
        float dist = sqrt(dx * dx + dy * dy);

        if (dist < minDist && dist <= m_searchRadius) {
            minDist = dist;
            closestIdx = static_cast<int>(i);
        }
    }

    return closestIdx;
}

float PointMatcher::calculateMatchConfidence(const std::vector<cv::DMatch>& matches,
    int selectedMatchIdx) {
    if (matches.empty() || selectedMatchIdx < 0) {
        return 0.0f;
    }

    // Find the selected match
    cv::DMatch selectedMatch;
    bool found = false;
    for (const auto& match : matches) {
        if (match.queryIdx == selectedMatchIdx) {
            selectedMatch = match;
            found = true;
            break;
        }
    }

    if (!found) {
        return 0.0f;
    }

    // For ORB with Hamming distance, lower distance is better
    // Typical range: 0-100 for good matches, >150 for poor matches
    // Normalize to 0-1 where 1 is best
    float normalizedDistance = std::max(0.0f, 1.0f - (selectedMatch.distance / 150.0f));

    // Confidence increases with more matches and better distance
    float matchCountFactor = std::min(1.0f, static_cast<float>(matches.size()) / 20.0f);
    float confidence = (normalizedDistance * 0.7f) + (matchCountFactor * 0.3f);

    return std::max(0.0f, std::min(1.0f, confidence));
}

bool PointMatcher::verifyMatch(const cv::Mat& frame1, const cv::Mat& frame2,
    const QPointF& point1, const QPointF& point2) {
    // Extract small patches around the points for verification
    int patchSize = 31; // Must be odd
    int halfSize = patchSize / 2;

    // Check bounds
    if (point1.x() < halfSize || point1.y() < halfSize ||
        point1.x() >= frame1.cols - halfSize || point1.y() >= frame1.rows - halfSize ||
        point2.x() < halfSize || point2.y() < halfSize ||
        point2.x() >= frame2.cols - halfSize || point2.y() >= frame2.rows - halfSize) {
        Logger::instance().log("Match verification failed: points too close to image border", "WARNING");
        return false;
    }

    cv::Rect roi1(static_cast<int>(point1.x()) - halfSize,
        static_cast<int>(point1.y()) - halfSize,
        patchSize, patchSize);
    cv::Rect roi2(static_cast<int>(point2.x()) - halfSize,
        static_cast<int>(point2.y()) - halfSize,
        patchSize, patchSize);

    cv::Mat patch1 = frame1(roi1).clone();
    cv::Mat patch2 = frame2(roi2).clone();

    // Convert to grayscale if needed
    if (patch1.channels() == 3) {
        cv::cvtColor(patch1, patch1, cv::COLOR_BGR2GRAY);
    }
    if (patch2.channels() == 3) {
        cv::cvtColor(patch2, patch2, cv::COLOR_BGR2GRAY);
    }

    // Compute normalized cross-correlation
    cv::Mat result;
    cv::matchTemplate(patch1, patch2, result, cv::TM_CCORR_NORMED);

    double correlation = result.at<float>(0, 0);

    Logger::instance().log(QString("Match verification correlation: %1").arg(correlation), "DEBUG");

    return correlation > 0.5; // Threshold for acceptance
}

bool PointMatcher::findCorrespondingPoint(const cv::Mat& sourceFrame,
    const cv::Mat& targetFrame,
    const QPointF& sourcePoint,
    QPointF& correspondingPoint) {

    Logger::instance().log(QString("Finding corresponding point for source point (%1, %2)")
        .arg(sourcePoint.x()).arg(sourcePoint.y()), "INFO");

    m_lastMatchConfidence = 0.0f;

    if (sourceFrame.empty() || targetFrame.empty()) {
        QString msg = "Source or target frame is empty";
        Logger::instance().log(msg, "CRITICAL");
        emit matchFailed(msg);
        return false;
    }

    try {
        // Convert to grayscale if needed
        cv::Mat graySource, grayTarget;
        if (sourceFrame.channels() == 3) {
            cv::cvtColor(sourceFrame, graySource, cv::COLOR_BGR2GRAY);
        }
        else {
            graySource = sourceFrame;
        }

        if (targetFrame.channels() == 3) {
            cv::cvtColor(targetFrame, grayTarget, cv::COLOR_BGR2GRAY);
        }
        else {
            grayTarget = targetFrame;
        }

        // Detect keypoints and compute descriptors using ORB
        std::vector<cv::KeyPoint> keypointsSource, keypointsTarget;
        cv::Mat descriptorsSource, descriptorsTarget;

        Logger::instance().log("Detecting ORB keypoints in source frame...", "INFO");
        m_orbDetector->detectAndCompute(graySource, cv::noArray(),
            keypointsSource, descriptorsSource);

        Logger::instance().log("Detecting ORB keypoints in target frame...", "INFO");
        m_orbDetector->detectAndCompute(grayTarget, cv::noArray(),
            keypointsTarget, descriptorsTarget);

        if (keypointsSource.empty() || keypointsTarget.empty()) {
            QString msg = QString("No keypoints detected (source: %1, target: %2)")
                .arg(keypointsSource.size()).arg(keypointsTarget.size());
            Logger::instance().log(msg, "WARNING");
            emit matchFailed(msg);
            return false;
        }

        if (descriptorsSource.empty() || descriptorsTarget.empty()) {
            QString msg = "No descriptors computed";
            Logger::instance().log(msg, "WARNING");
            emit matchFailed(msg);
            return false;
        }

        Logger::instance().log(QString("Detected %1 keypoints in source, %2 in target")
            .arg(keypointsSource.size()).arg(keypointsTarget.size()), "INFO");

        // Find the closest keypoint to the clicked point in source frame
        int closestIdx = findClosestKeypoint(keypointsSource, sourcePoint);

        if (closestIdx == -1) {
            QString msg = QString("No keypoint found within %1 pixels of clicked point")
                .arg(m_searchRadius);
            Logger::instance().log(msg, "WARNING");
            emit matchFailed(msg);
            return false;
        }

        float distToClick = sqrt(
            pow(keypointsSource[closestIdx].pt.x - sourcePoint.x(), 2) +
            pow(keypointsSource[closestIdx].pt.y - sourcePoint.y(), 2)
        );

        Logger::instance().log(QString("Found keypoint at distance %1 pixels from click")
            .arg(distToClick), "INFO");

        // Match descriptors using BFMatcher
        std::vector<cv::DMatch> matches;
        m_matcher->match(descriptorsSource, descriptorsTarget, matches);

        if (matches.empty()) {
            QString msg = "No matches found";
            Logger::instance().log(msg, "WARNING");
            emit matchFailed(msg);
            return false;
        }

        Logger::instance().log(QString("Found %1 matches").arg(matches.size()), "INFO");

        // Find the match for our selected keypoint
        bool matchFound = false;
        for (const auto& match : matches) {
            if (match.queryIdx == closestIdx) {
                cv::Point2f matchedPt = keypointsTarget[match.trainIdx].pt;
                correspondingPoint = QPointF(matchedPt.x, matchedPt.y);

                // Calculate confidence
                m_lastMatchConfidence = calculateMatchConfidence(matches, closestIdx);

                Logger::instance().log(QString("Corresponding point found at (%1, %2) with confidence %3")
                    .arg(correspondingPoint.x()).arg(correspondingPoint.y())
                    .arg(m_lastMatchConfidence), "INFO");

                // Verify match quality
                if (m_lastMatchConfidence < m_minMatchQuality) {
                    Logger::instance().log(QString("Match confidence %1 below threshold %2")
                        .arg(m_lastMatchConfidence).arg(m_minMatchQuality), "WARNING");
                    emit matchFailed(QString("Low confidence match: %1").arg(m_lastMatchConfidence));
                    return false;
                }

                // Verify using patch correlation
                if (!verifyMatch(sourceFrame, targetFrame, sourcePoint, correspondingPoint)) {
                    Logger::instance().log("Match failed patch verification", "WARNING");
                    emit matchFailed("Match failed patch verification");
                    return false;
                }

                matchFound = true;
                emit this ->matchFound(correspondingPoint, m_lastMatchConfidence);
                break;
            }
        }

        if (!matchFound) {
            QString msg = "No match found for selected keypoint";
            Logger::instance().log(msg, "WARNING");
            emit matchFailed(msg);
            return false;
        }

        return true;

    }
    catch (const cv::Exception& e) {
        QString msg = QString("OpenCV error in feature matching: %1").arg(e.what());
        Logger::instance().log(msg, "CRITICAL");
        emit matchFailed(msg);
        return false;
    }
}

void PointMatcher::drawDebugVisualization(const cv::Mat& frame1, const cv::Mat& frame2,
    const QPointF& point1, const QPointF& point2,
    const std::vector<cv::KeyPoint>& kp1,
    const std::vector<cv::KeyPoint>& kp2,
    const std::vector<cv::DMatch>& matches) {
    // This can be used for debugging - draw matches between frames
    cv::Mat imgMatches;
    cv::drawMatches(frame1, kp1, frame2, kp2, matches, imgMatches);

    // Draw the selected points
    cv::circle(imgMatches, cv::Point(point1.x(), point1.y()), 10, cv::Scalar(0, 255, 0), 3);
    cv::circle(imgMatches, cv::Point(frame1.cols + point2.x(), point2.y()), 10, cv::Scalar(0, 255, 0), 3);

    // Save for debugging
    cv::imwrite("debug_matches.jpg", imgMatches);
    Logger::instance().log("Debug visualization saved to debug_matches.jpg", "DEBUG");
}