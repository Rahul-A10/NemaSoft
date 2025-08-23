#include "utils.h"

#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QDir>
#include <QMutex>
#include <QDebug>
#include <QFileInfo>
#include <QCoreApplication>
#include <iostream>


static bool camDebug = false;
static bool fpsDebug = false;

void set_camDebug_flag(bool val) { camDebug = val; }
bool get_camDebug_flag() { return camDebug; }

void set_fpsDebug_flag(bool val) { fpsDebug = val; }
bool get_fpsDebug_flag() { return fpsDebug; }


//Logger class

Q_LOGGING_CATEGORY(logApp, "myapp.application")

// Static members for logger configuration
static QString s_logFileName = "application.log";
static QString s_logFilePath = "logs/application.log";
static qint64 s_maxFileSize = 1 * 1024 * 1024; // 1MB default
static QtMsgType s_minLogLevel = QtDebugMsg;
static bool s_consoleOutput = true;
static QMutex s_logMutex;
static QFile* s_logFile = nullptr;
static bool s_initialized = false;

// Custom message handler that writes to file
void Logger::fileMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    // Create logs directory if it doesn't exist
    QDir().mkpath("logs");

    static QFile file("logs/application.log");
    static bool fileOpened = file.open(QIODevice::WriteOnly | QIODevice::Append);

    if (fileOpened) {
        QTextStream stream(&file);

        // Format timestamp
        QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");

        // Convert message type to string
        QString typeStr;
        switch (type) {
        case QtDebugMsg:    typeStr = "DEBUG"; break;
        case QtInfoMsg:     typeStr = "INFO"; break;
        case QtWarningMsg:  typeStr = "WARNING"; break;
        case QtCriticalMsg: typeStr = "CRITICAL"; break;
        case QtFatalMsg:    typeStr = "FATAL"; break;
        }

        // Write formatted message
        stream << QString("[%1] [%2] [%3] %4")
            .arg(timestamp, typeStr, context.category, msg) << Qt::endl;
        stream.flush();
    }
}


void Logger::initialize()
{
    QMutexLocker locker(&s_logMutex);

    if (s_initialized) {
        return; // Already initialized
    }

    // Create logs directory
    QDir().mkpath("logs");

    // Remove existing log file if it exists
    QFile existingFile(s_logFilePath);
    if (existingFile.exists()) {
        existingFile.remove();
    }

    // Create and open log file
    s_logFile = new QFile(s_logFilePath);
    if (!s_logFile->open(QIODevice::WriteOnly | QIODevice::Append)) {
        delete s_logFile;
        s_logFile = nullptr;
        qWarning() << "Failed to open log file:" << s_logFilePath;
        return;
    }

    // Install our custom message handler
    qInstallMessageHandler(fileMessageHandler);
    s_initialized = true;
}

void Logger::cleanup()
{
    QMutexLocker locker(&s_logMutex);

    if (s_logFile) {
        s_logFile->close();
        delete s_logFile;
        s_logFile = nullptr;
    }

    // Restore default Qt message handler
    qInstallMessageHandler(nullptr);
    s_initialized = false;
}


std::vector<int> checkAvailableCameraConnections() {
    LOG_INFO("Searching for available camera indices...");

	bool noneFound = true;
	std::vector<int> availableCameras;

    // Loop through potential indices to see which ones are valid
    for (int i = 0; i < 10; ++i) {
        cv::VideoCapture cap(i);

        if (cap.isOpened()) {
			noneFound = false;

            //cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
            //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
            //cap.set(cv::CAP_PROP_FPS, 5);

            cv::Mat frame;
            cap >> frame;
            if (!frame.empty()) {
                cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
                cv::imwrite("cam_" + std::to_string(i) + ".jpg", frame);
				availableCameras.push_back(i);
            }

            LOG_INFO("Camera" << i << "opened" << (frame.empty() ? "but frame empty" : "successfully"));

            cap.release();
        }
    }

	assert(!noneFound && "No cameras found at indices 0-9. Please connect a camera and try again.");

    // check if not in cam debug mode, then we expect min of 3 cameras
    if (!get_camDebug_flag())
        assert(availableCameras.size() >= 3 && "expected number of camera is less than 3.");

	return availableCameras;
}


cv::Mat cropInputImage(const cv::Mat& input) {
    cv::Mat gray;
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);

    // Threshold to find non-black regions
    cv::Mat mask;
    cv::threshold(gray, mask, 10, 255, cv::THRESH_BINARY);

    // Find contours of non-black area
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return input.clone(); // nothing to crop
    }

    // Get bounding rect of the largest contour
    int maxIdx = 0;
    double maxArea = 0.0;
    for (int i = 0; i < contours.size(); i++) {
        double a = cv::contourArea(contours[i]);
        if (a > maxArea) {
            maxArea = a;
            maxIdx = i;
        }
    }

    cv::Rect bbox = cv::boundingRect(contours[maxIdx]);
    cv::Mat cropped = input(bbox);

    // Ensure dimensions divisible by 4
    int newW = cropped.cols - (cropped.cols % 4);
    int newH = cropped.rows - (cropped.rows % 4);
    cv::Rect finalRect(0, 0, newW, newH);
    cropped = cropped(finalRect).clone();

    return cropped;
}
