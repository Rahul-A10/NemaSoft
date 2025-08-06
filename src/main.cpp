#include <QApplication>
#include "mainwindow.h"
#include "utils.h"

int main(int argc, char* argv[]) {

    set_camDebug_flag(true);
	set_fpsDebug_flag(true);

    // Initialize logger
    Logger::initialize(); 

    LOG_INFO("Application starting up: " << (get_camDebug_flag() ? "reading image input" : "reading video input"));

    std::vector<int> cams = checkAvailableCameraConnections();
	// TODO: ask the user to set the camera index for arducam and duocam based on the printed cam outputs...
	// if some cam not available, then ask to retry... call above method to check again ??

    QApplication app(argc, argv);
    MainWindow w;
    w.show(); 
    int result = app.exec();

    LOG_INFO("Application shutting down");
    Logger::cleanup(); // Clean shutdown
    return result;
}