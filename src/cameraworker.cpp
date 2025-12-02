#include "cameraworker.h"
#include <QThread>
#include "logger.h"
#include <filesystem>
#include "utils.h"

CameraWorker::CameraWorker(int camIndex, int camType,
    int frameWidth, int frameHeight, int fps,
    QObject* parent)
    : QObject(parent)
    , m_running(false)  // Start as false, will be set true in process()
{
    m_cameraIndex = camIndex;
    m_cameraType = camType;
    Logger::info(QString("CameraWorker initialized with camera index: %1 and type: %2").arg(m_cameraIndex).arg(m_cameraType));

    if (camIndex != IMG) {
        m_cap.open(m_cameraIndex);
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
    m_running.store(false, std::memory_order_release);  // Atomic, no mutex needed
    Logger::info("CameraWorker stop requested");
}

void CameraWorker::start() {
    m_running.store(true, std::memory_order_release);
    Logger::info("CameraWorker start requested");
}

void CameraWorker::process() {
    Logger::info("CameraWorker process loop starting");
    m_running.store(true, std::memory_order_release);  // Set running at start

    while (m_running.load(std::memory_order_acquire)) {
        cv::Mat frame;

        // Capture frame
        if (m_cameraIndex == IMG) {
            std::string imgPath = "test_img.png";
            if (!std::filesystem::exists(imgPath)) {
                Logger::critical(QString("Test Image file does not exist: %1")
                    .arg(QString::fromStdString(imgPath)));
                QThread::msleep(1000);
                continue;
            }
            frame = cv::imread(imgPath);
        }
        else {
            m_cap >> frame;
        }

        if (frame.empty()) {
            Logger::warning("Empty frame captured from camera");
            QThread::msleep(50);
            continue;
        }

        // Check if we should stop before processing
        if (!m_running.load(std::memory_order_acquire)) {
            break;
        }

        // Update current frame (thread-safe)
        {
            QMutexLocker locker(&m_frameMutex);
            m_currentFrame = frame.clone();
        }

        // Handle capture flag
        {
            QMutexLocker locker(&m_mutex);
            if (m_captureImg) {
                Logger::info("Frame captured and stored");
                m_capturedFrame = frame.clone();
                m_captureImg = false;
            }
        }

        // Check again before emitting (avoid emitting after stop requested)
        if (!m_running.load(std::memory_order_acquire)) {
            break;
        }

        // Emit the frame for display
        QImage qImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_BGR888);
        emit frameReady(qImage.copy(), m_cameraType);

        QThread::msleep(50);
    }

    Logger::info("CameraWorker process loop exited, quitting thread");

    // CRITICAL: Exit the thread's event loop
    QThread::currentThread()->quit();
}