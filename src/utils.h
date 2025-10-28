#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <opencv2/opencv.hpp>

#include <filesystem>
#include <fstream>

#include <QString>
#include <QLoggingCategory>
#include <QDebug>
#include <QTextEdit>
#include <QDateTime>
#include <QTextCursor>

enum cameraType {
	NONE = -1,
	ARDUCAM,
	MICROCAM1,
	MICROCAM2
};

enum cameraIndex {
	IMG = -1,
	WEBCAM
	// add other USB slots here 
};


// Timer macros for measuring execution time
#define START_TIMER(name) auto start_##name = std::chrono::high_resolution_clock::now()
#define END_TIMER(name) auto end_##name = std::chrono::high_resolution_clock::now(); \
                            auto duration_##name = std::chrono::duration_cast<std::chrono::milliseconds>(end_##name - start_##name); \
                            Logger::debug(QString("[TIMER] %1: %2 ms").arg(#name).arg(duration_##name.count()));


// functions declarations
void set_camDebug_flag(bool val);
bool get_camDebug_flag();
void set_fpsDebug_flag(bool val);
bool get_fpsDebug_flag();
std::vector<int> checkAvailableCameraConnections();



//// Logger class
//class Logger
//{
//public:
//	/*static void initialize();
//	static void cleanup();
//	static void appendLog(QTextEdit* logTextEdit, const QString& message, const QString& level = "INFO");*/
//
//private:
//	//static void fileMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg);
//};

cv::Scalar getColorForClass(int classId);


// General application logging
//Q_DECLARE_LOGGING_CATEGORY(logApp)
//
//#define LOG_INFO(...) qCInfo(logApp)<< __VA_ARGS__
//#define LOG_WARNING(...) qCWarning(logApp) << __VA_ARGS__
//#define LOG_CRITICAL(...) qCCritical(logApp) << __VA_ARGS__

#endif