#include "utils.h"
#include "logger.h"
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




std::vector<int> checkAvailableCameraConnections() {
    Logger::info("Searching for available camera indices...");

	bool noneFound = true;
	std::vector<int> availableCameras;

    // Loop through potential indices to see which ones are valid
    for (int i = 0; i < 4; ++i) {
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

            Logger::info(QString("Camera %1 opened %2 but frame empty: successfully").arg(i).arg(frame.empty()));

            cap.release();
        }
    }

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


cv::Scalar getColorForClass(int classId) {
    static std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),    // Blue
        cv::Scalar(0, 255, 0),    // Green
        cv::Scalar(0, 0, 255),    // Red
        cv::Scalar(255, 255, 0),  // Cyan
        cv::Scalar(255, 0, 255),  // Magenta
        cv::Scalar(0, 255, 255),  // Yellow
        cv::Scalar(128, 0, 128),  // Purple
        cv::Scalar(255, 165, 0),  // Orange
        cv::Scalar(0, 128, 128),  // Teal
        cv::Scalar(128, 128, 0)   // Olive
    };
    return colors[classId % colors.size()];
}

