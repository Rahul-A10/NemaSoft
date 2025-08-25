#ifndef DETECTIONTRAVERSER_H
#define DETECTIONTRAVERSER_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <opencv2/opencv.hpp>
#include "XYZStage.h"

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
    void traversalStarted();
    void updateProgress(int current, int total);
    void waitingForUserAdjustment();
    void traversalFinished(const QString& message);

private:
    XYZStage* m_xyzStage;
    std::vector<cv::Rect> m_macroImgPath;
    cv::Mat m_transformMatrix;

    QMutex m_mutex;
    QWaitCondition m_pauseCondition;
    bool m_paused;
    bool m_aborted;
};

#endif // DETECTIONTRAVERSER_H