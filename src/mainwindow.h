#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QLabel>
#include <QTimer>
#include <QGroupBox>
#include <QPushButton>
#include <QElapsedTimer>
#include <opencv2/opencv.hpp>

#include "cameraworker.h"
#include "inferenceworker.h"
#include <QLineEdit>
#include <QGraphicsPixmapItem>
#include "ZoomableGraphicsView.h"
#include "XYZStage.h"


struct cameraOp
{
    QThread* thrd;
    CameraWorker* camWorker;
    QElapsedTimer FPSTimer;
    int frameCount;
    QPushButton* cameraBtn;

    void toggleCamera() {
        camWorker->stop();
        thrd->quit();
        thrd->wait();
        camWorker = nullptr;
        thrd->deleteLater();
        thrd = nullptr;
    };
};

struct inferenceOp
{
    QThread* thrd;
    InferenceWorker* infWorker;

    void free() {
        thrd->quit();
        thrd->wait();
        infWorker = nullptr;
        thrd->deleteLater();
        thrd = nullptr;
    }
};

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

    void resizeEvent(QResizeEvent* event);

    QGroupBox* setupMovementUI();
    QGroupBox* setupPositionUI();
    QGroupBox* setupControlUI();

    // might need to change this return type
    QLabel* setupArducamUI();
    QLabel* setupDuocamUI();

    void updateFrame(const QImage& img, int camType);
    void renderLatestFrame();

    void onStartArducam();
    void onCaptureMacroImg();
    void inferenceResult(const cv::Mat& frame, const std::vector<cv::Rect>& boxCentroids);
    void onPredictMacroImg();


    void onGoToPosition1();
    void onStartDuocam();
    void onCaptureMicroImg();
    void onPredictMicroImg();

    void onLeftFastClicked();
    void onLeftSlowClicked();
    void onRightFastClicked();
    void onRightSlowClicked();
    void onUpFastClicked();
    void onUpSlowClicked();
    void onDownFastClicked();
    void onDownSlowClicked();
    void onZUpClicked();
    void onZUpFastClicked();
    void onZDownClicked();
    void onZDownFastClicked();
    void onSlant1Clicked();
    void onSlant2Clicked();
    void onSlant3Clicked();
    void onSlant4Clicked();
    void updatePositionDisplay();
    void setupTransformationMatrix();
    void onAbortPathClicked();
	void onResumePathClicked();

private:
    // Transformation methods
    cv::Mat calculateTransformationMatrix(const std::vector<cv::Point2f>& imagePoints,
        const std::vector<cv::Point2f>& realPoints);
    std::vector<cv::Point2f> convertImageToRealCoordinates(const std::vector<cv::Rect>& imageBoundingBoxes,
        const cv::Mat& transformMatrix);
    std::vector<cv::Point2f> convertMacroImagePathToReal(const cv::Mat& transformMatrix);
    void traverseRealCoordinatePath(const cv::Mat& transformMatrix);



    // private class members

    cv::Mat m_transformMatrix;

    QLabel* m_xLabel;
    QLabel* m_yLabel;
    QLabel* m_zLabel;
    QTimer* m_positionUpdateTimer;
    double m_prevX;
    double m_prevY;
    double m_prevZ;
    QLineEdit* m_x1;
    QLineEdit* m_y1;
    QLineEdit* m_z1;
    QLineEdit* m_stepEdit;
    bool abort = false;
	bool pause = false;

    QGroupBox* m_controlGroup = nullptr;

	ZoomableGraphicsView* m_arducamView = nullptr;
    QGraphicsScene* m_arducamScene = nullptr;
    QGraphicsPixmapItem* m_arducamPixmapItem = nullptr;
    QLabel* m_arducamFPS = nullptr;
    cameraOp m_arducamOp;
    inferenceOp m_macroImgInference;
	QImage m_latestArducamImage;
	cv::Mat m_currentMacroImg;
	std::vector<cv::Rect> m_macroImgPath;

    ZoomableGraphicsView* m_microCam1View = nullptr;
    QGraphicsScene* m_microCam1Scene = nullptr;
    QGraphicsPixmapItem* m_microCam1PixmapItem = nullptr;
	QImage m_latestMicroCam1Image;
    cv::Mat m_currentMicroImg1;
    QLabel* m_microCam1FPS = nullptr;
    cameraOp m_microCam1Op;

    ZoomableGraphicsView* m_microCam2View = nullptr;
    QGraphicsScene* m_microCam2Scene = nullptr;
    QGraphicsPixmapItem* m_microCam2PixmapItem = nullptr;
    QImage m_latestMicroCam2Image;
    cv::Mat m_currentMicroImg2;
    QLabel* m_microCam2FPS = nullptr;
    cameraOp m_microCam2Op;

	QMutex m_frameMutex;
	QLabel* m_uiFPS = nullptr;
    QElapsedTimer m_UITimer;
    int m_uiFrameCount;

	// TODO: let user set these values before initializing the UI in main.cpp
    //int m_arducamIndex = 1;
	//int m_microCam1Index = 2;
	//int m_microCam2Index = 3;

	XYZStage m_xyzStage;
	XYZStage n_xyzStage;
};

#endif // MAINWINDOW_