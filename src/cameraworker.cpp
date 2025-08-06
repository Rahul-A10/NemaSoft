#include "cameraworker.h"
#include <QThread>

#include <filesystem>
#include "utils.h"

CameraWorker::CameraWorker(int camIndex, int camType, QObject* parent)
    : QObject(parent) {
    m_cameraIndex = camIndex;
    m_cameraType = camType;

    LOG_INFO("CameraWorker initialized with camera index: " << m_cameraIndex << " and type: " << m_cameraType);

    if (camIndex != IMG) {
        m_cap.open(m_cameraIndex);
        //m_cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
        //m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
        //m_cap.set(cv::CAP_PROP_FPS, 12);
        if (!m_cap.isOpened()) {
            LOG_CRITICAL("Failed to open camera with index: " << m_cameraIndex);
            return;
        }

        m_frameWidth = m_cap.get(cv::CAP_PROP_FRAME_WIDTH);
        m_frameHeight = m_cap.get(cv::CAP_PROP_FRAME_HEIGHT);

		    LOG_INFO("Camera opened with resolution: " << m_frameWidth << "x" << m_frameHeight);
    }
    else {
        m_frameWidth = 1280;
        m_frameHeight = 720;
    }

    m_running = true;
}

CameraWorker::~CameraWorker() {
	LOG_INFO("deleting CameraWorker object");
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
            // TODO: this exits the thrd if stop() is called. we might want to modify this to not exit the loop but also don't process further?
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
                std::string imgPath = "test_img2.jpg";
                if (!std::filesystem::exists(imgPath)) {
                    LOG_CRITICAL("Test Image file does not exist: " << imgPath);
                    return;
                }

                frame = cv::imread(imgPath);
                //LOG_INFO("Reading image: " << imgPath << "dims: " << frame.cols << "x" << frame.rows);
            }
            else
                m_cap >> frame;

            if (frame.empty()) {
                //std::cerr << "something wrong" << std::endl;
                LOG_WARNING("Empty frame captured from camera");
                continue;
            }

            //cv::flip(frame, frame, 1);
            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

            if (getCaptureImg() || m_cameraIndex == IMG) {
                LOG_INFO("Captured frame");
                setCapturedFrame(frame);
            }

            // for large images, resizing helps with UI FPS
            if (frame.cols > 1280 || frame.rows > 720)
                cv::resize(frame, frame, cv::Size(1280, 720));

            QImage qImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
            emit frameReady(qImage.copy(), m_cameraType);

            QThread::msleep(30);
        }
    }
}