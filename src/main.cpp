#include <QApplication>
#include "mainwindow.h"
#include "utils.h"
#include "QStyleFactory"
#include "QPalette"
#include "logger.h"
int main(int argc, char* argv[]) {

    set_camDebug_flag(true);
	set_fpsDebug_flag(true);

    // Initialize logger
 
    Logger::info(QString("Application starting up: %1").arg(get_camDebug_flag() ? "reading image input" : "reading video input"));

    std::vector<int> cams = checkAvailableCameraConnections();
	// TODO: ask the user to set the camera index for arducam and duocam based on the printed cam outputs...
	// if some cam not available, then ask to retry... call above method to check again ??
    
    QApplication app(argc, argv);
    app.setStyle(QStyleFactory::create("Fusion"));
    QString iconPath = QCoreApplication::applicationDirPath() + "/logo.png";
    app.setWindowIcon(QIcon(iconPath));

    // Create dark palette
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);
    darkPalette.setColor(QPalette::Disabled, QPalette::Text, Qt::darkGray);
    darkPalette.setColor(QPalette::Disabled, QPalette::ButtonText, Qt::darkGray);

    app.setPalette(darkPalette);
    MainWindow w;
    w.show(); 
    int result = app.exec();

    Logger::info("Application shutting down");
    //Logger::cleanup(); // Clean shutdown
    return result;
}