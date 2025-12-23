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
#include "DetectionTraverser.h"
#include <QTextEdit>
#include "logger.h"
#include "logdisplay.h"
#include "PointMatcher.h"
#include "StageMapping.h"
#include "DetectionTraverser.h"
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

struct YoloAnnotation {
    int classId;
    QPointF center;  // Normalized center coordinates (0-1)
    double width;    // Normalized width (0-1)
    double height;   // Normalized height (0-1)
};





class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();
    QLineEdit* m_z_injection;
    void resizeEvent(QResizeEvent* event);

    QGroupBox* setupMovementUI();
    QGroupBox* setupPositionUI();
    QGroupBox* setupControlUI();

    // might need to change this return type
    //QLabel* setupArducamUI();
    //QLabel* setupDuocamUI();

    


    void renderLatestFrame();

    void onStartArducam();
    void onCaptureMacroImg();
    void inferenceResult(const cv::Mat& frame, const std::vector<cv::Rect>& boxCentroids);
    void onPredictMacroImg();
	void onCaptureMacroData();
    void onCaptureMicroData();


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
	void setMovementControlsEnabled(bool enabled);
    void onTraversalStarted();
    void onWaitingForUser();
    void onConfirmAdjustmentClicked();
    void onTraversalFinished(const QString& message);
	void onHomeClicked();
    void onArducamClicked(const QPointF& scenePos, const QPointF& imagePos);
    void onMicroCam1Clicked(const QPointF& scenePos, const QPointF& imagePos);
    void onMicroCam2Clicked(const QPointF& scenePos, const QPointF& imagePos);
	void clearMacroAnnotations();
	void clearMicroAnnotations();
    int getSelectedMacroId() const;
    QString getSelectedMacroLabel() const;
	QString getSelectedMicroLabel() const;
    int getSelectedMicroId() const;
    void ImageDisplay(cameraType);
	void updateFrame(const QImage& img, int camType);
    //void drawAnnotationBox(int index);
    bool isClickInsideBox(const QPointF& imagePos, const YoloAnnotation& ann, int imageWidth, int imageHeight);
    void log(const QString& message, const QString& level = "INFO");
	void onTriversePath();
    void drawAnnotations(cv::Mat& img, const QVector<YoloAnnotation>& annotations, QComboBox* combobox);
    void onMicroCam1FrameReady(const QImage& img, int camType);
    void onMicroCam2FrameReady(const QImage& img, int camType);
    void onPointMatchFound(QPointF targetPoint, float confidence);

    void onPointMatchFailed(const QString& reason);
	void onFocusCam1(); 
    void onFocusCam2();
    

private:
    // Camera configuration constants
    static constexpr int ARDUCAM_ID = 0;// replace id with IMG to desplay static image
    static constexpr int MICROCAM1_ID = 2;
    static constexpr int MICROCAM2_ID = 1;


    static constexpr int ARDUCAM_WIDTH = 3840;
    static constexpr int ARDUCAM_HEIGHT = 2160;
    static constexpr int ARDUCAM_FPS = 10;

    static constexpr int MICROCAM_WIDTH = 2720;
    static constexpr int MICROCAM_HEIGHT = 1536;
    static constexpr int MICROCAM_FPS = 8;
    // Crosshair positions (normalized 0-1 coordinates)
    static constexpr double ARDUCAM_CROSSHAIR_X = 0.5;
    static constexpr double ARDUCAM_CROSSHAIR_Y = 0.5;

    static constexpr double MICROCAM1_CROSSHAIR_X = 0.51;
    static constexpr double MICROCAM1_CROSSHAIR_Y = 0.41;

    static constexpr double MICROCAM2_CROSSHAIR_X = 0.49;
    static constexpr double MICROCAM2_CROSSHAIR_Y = 0.35;

    QPointF m_arducamCrosshairPos;      // Center by default
    QPointF m_microCam1CrosshairPos;    // Custom position
    QPointF m_microCam2CrosshairPos;    // Custom position

    // Helper function
    QImage drawCrosshairOnImage(const QImage& img, const QPointF& normalizedPos,
        const QColor& color = Qt::green, int size = 20, int thickness = 2);


    StageMapping* mapper = nullptr;
	StageMapping* mapper2 = nullptr;
    std::atomic<bool> m_microCam1Stopping{ false };
    std::atomic<bool> m_microCam2Stopping{ false };

    std::vector<PointMapping> frame1_points = {
        {530, 500, 65579, 26727},
        {456, 915, 65000, 25886},
        {1658, 1369, 63000, 26272},
        {2272, 642, 63386, 28272},
    };

    // Frame 2 calibration data (example, replace with real measurements)
    std::vector<PointMapping> frame2_points = {
        {479, 1372, 65579, 26727},
        {1121, 1466, 65000, 25886},
        {1991, 799, 63000, 26272},
        {903, 311, 63386, 28272},
    };

    
    // Transformation methods
    cv::Mat calculateTransformationMatrix(const std::vector<cv::Point2f>& imagePoints,
        const std::vector<cv::Point2f>& realPoints);
    LogDisplay* m_logDisplay;
    
    
    // private class members
    cv::Mat m_transformMatrix;
    QVector<YoloAnnotation> m_macroAnnotations;
    QVector<YoloAnnotation> m_microAnnotations1;
    QVector<YoloAnnotation> m_microAnnotations2;
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
    QMap<QGraphicsRectItem*, int> m_annotationRects; // Maps rect items to annotation indices
    QLabel* m_arducamFPS = nullptr;
    cameraOp m_arducamOp;
    inferenceOp m_macroImgInference;
	QImage m_latestArducamImage;
	cv::Mat m_currentMacroImg;
    cv::Mat m_currentMacroImgdata;
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

    // Movement control buttons
    QPushButton* m_leftFastBtn = nullptr;
    QPushButton* m_leftSlowBtn = nullptr;
    QPushButton* m_rightFastBtn = nullptr;
    QPushButton* m_rightSlowBtn = nullptr;
    QPushButton* m_upFastBtn = nullptr;
    QPushButton* m_upSlowBtn = nullptr;
    QPushButton* m_downFastBtn = nullptr;
    QPushButton* m_downSlowBtn = nullptr;
    QPushButton* m_zUpBtn = nullptr;
    QPushButton* m_zUpFastBtn = nullptr;
    QPushButton* m_zDownBtn = nullptr;
    QPushButton* m_zDownFastBtn = nullptr;
    QPushButton* m_slant1Btn = nullptr;
    QPushButton* m_slant2Btn = nullptr;
    QPushButton* m_slant3Btn = nullptr;
    QPushButton* m_slant4Btn = nullptr;
	//Position display and go to position
    QPushButton* m_goToPositionBtn = nullptr;
    QPushButton* m_confirmAdjustmentBtn = nullptr;
	QPushButton* m_Focus_cam1 = nullptr;
    QPushButton* m_Focus_cam2 = nullptr;
	QPushButton* m_abortPathBtn = nullptr;
    QPushButton* m_resumePathBtn = nullptr;
	QPushButton* m_homeBtn = nullptr;
    QComboBox* m_macroComboBox;
    QComboBox* m_microComboBox;
    QComboBox* m_trivarsePath;
    QPushButton* m_predictMicroImg = nullptr;

    QThread* m_traverserThread = nullptr;
    DetectionTraverser* m_traverser = nullptr;
    PointMatcher* m_pointMatcher;
    
	
};



#endif // MAINWINDOW_