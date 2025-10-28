#include "cameraworker.h"
#include <QThread>
#include "logger.h"
#include <filesystem>
#include "utils.h"

CameraWorker::CameraWorker(int camIndex, int camType,
    int frameWidth, int frameHeight, int fps,
    QObject* parent)
    : QObject(parent)
{
    m_cameraIndex = camIndex;
    m_cameraType = camType;
    Logger::info(QString("CameraWorker initialized with camera index: %1 and type: %2").arg(m_cameraIndex).arg(m_cameraType));
    if (camIndex != IMG) {
        m_cap.open(m_cameraIndex);
        // Apply user-specified settings
        m_cap.set(cv::CAP_PROP_FRAME_WIDTH, frameWidth);
        m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, frameHeight);
        m_cap.set(cv::CAP_PROP_FPS, fps);
        if (!m_cap.isOpened()) {
            Logger::critical(QString("Failed to open camera with index: %1").arg(m_cameraIndex));
            return;
        }
        m_frameWidth = m_cap.get(cv::CAP_PROP_FRAME_WIDTH);
        m_frameHeight = m_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        Logger::info(QString("Camera opened with resolution: %1x%2 @ %3 FPS").arg(m_frameWidth).arg(m_frameHeight).arg(fps));
    }
    else {
        m_frameWidth = frameWidth;
        m_frameHeight = frameHeight;
    }
    m_running = true;
}

CameraWorker::~CameraWorker() {
    Logger::info("deleting CameraWorker object");
    stop();
    if (m_cap.isOpened()) {
        m_cap.release();
    }
    clearCapturedFrame();
}

void CameraWorker::stop() {
    QMutexLocker locker(&m_mutex);
    m_running = false;
}

void CameraWorker::start() {
    QMutexLocker locker(&m_mutex);
    //setCaptureImg(false); // reset the flag after restart
    m_running = true;
}

void CameraWorker::process() {
    while (true) {
        {
            QMutexLocker locker(&m_mutex);
            // TODO: this exits the thrd if stop() is called. might want to modify this to not exit the loop but also don't process further?
            if (!m_running) break;
        }
        cv::Mat frame;
        cv::Mat resized;
        if (!m_capturedFrame.empty()) {
            frame = m_capturedFrame;
            continue; // already rendered this frame
        }
        else {
            if (m_cameraIndex == IMG) {
                // TODO: make frame member of this class
                std::string imgPath = "test_img.png";
                if (!std::filesystem::exists(imgPath)) {
                    Logger::critical(QString("Test Image file does not exist: %1").arg(QString::fromStdString(imgPath)));
                    return;
                }
                frame = cv::imread(imgPath);
                //Logger::info(QString("Reading image: %1 dims: %2x%3").arg(QString::fromStdString(imgPath)).arg(frame.cols).arg(frame.rows));
            }
            else
                m_cap >> frame;
            if (frame.empty()) {
                //std::cerr << "something wrong" << std::endl;
                Logger::warning("Empty frame captured from camera");
                continue;
            }
            //cv::flip(frame, frame, 1);
            //cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
            if (getCaptureImg()) {
                Logger::info("Captured frame");
                setCapturedFrame(frame);
            }
            // for large images, resizing helps with UI FPS
            /*if (frame.cols > 1280 || frame.rows > 720)
                cv::resize(frame, frame, cv::Size(1280, 720));*/
            QImage qImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_BGR888);
            emit frameReady(qImage.copy(), m_cameraType);
            QThread::msleep(50);
        }
    }
}