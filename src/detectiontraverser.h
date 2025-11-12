#ifndef DETECTIONTRAVERSER_H
#define DETECTIONTRAVERSER_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <opencv2/opencv.hpp>
#include "XYZStage.h"
#include <vector>
#include "utils.h"
#include <QString>


class DetectionTraverser : public QObject
{
    Q_OBJECT

public:
    
    explicit DetectionTraverser(XYZStage* xyzStage, QObject *parent = nullptr);
    void setTraversalData(const std::vector<cv::Rect>& path, const cv::Mat& transformMatrix);

public slots:
    void process(); // The main worker function
    void abortTraversal();
    void userConfirmedAdjustment();

signals:
    void logMessage(const QString& message, const QString& level);
    void traversalStarted();
    void updateProgress(size_t current, size_t total);
    void waitingForUserAdjustment();
    void traversalFinished(const QString& message);

private:
    void log(const QString& message, const QString& level = "INFO") {
        emit logMessage(message, level);
    }
    
    XYZStage* m_xyzStage;
    std::vector<cv::Rect> m_macroImgPath;
    cv::Mat m_transformMatrix;

    QMutex m_mutex;
    QWaitCondition m_pauseCondition;
    bool m_paused;
    bool m_aborted;
	int MIN_X = 0;
	int MAX_X = 62000;
	int MIN_Y = 0;
	int MAX_Y = 150000;
	int min_z = 0;
	int MAX_Z = 150000;
};

#endif // DETECTIONTRAVERSER_H