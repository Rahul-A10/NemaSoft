#include "mainwindow.h"
#include "utils.h"

#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QFrame>
#include <QTextEdit>
#include <QObject>
#include <QTimer>
#include <QThread>
#include <QDebug>
#include <QDialog>
#include <QDir>
#include <QDateTime>
#include "XYZStage.h"
#include <opencv2/opencv.hpp>
// or more specific includes:
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {

    QWidget* centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    this->setMinimumWidth(1280);
    this->setMinimumHeight(720);

    int mainWidth = this->width();
    int mainHeight = this->height();

    QHBoxLayout* controlLayout = new QHBoxLayout();
    controlLayout->addWidget(setupMovementUI());
    controlLayout->addWidget(setupPositionUI());
    controlLayout->addWidget(setupControlUI());
    
    m_controlGroup = new QGroupBox();
    m_controlGroup->setLayout(controlLayout);
    m_controlGroup->setGeometry(0, 0, mainWidth * 0.5, mainHeight * 0.5);
    m_controlGroup->setMaximumWidth(mainWidth * 0.5);
    m_controlGroup->setMaximumHeight(mainHeight * 0.5);

    //Zoomable graphics view 
    m_arducamView = new ZoomableGraphicsView("Ardacam Output", this);
    m_arducamView->setZoomLimits(0.05, 10.0); // Set zoom limits for the view
    m_arducamView->setFrameStyle(QFrame::Box);
    m_arducamView->setGeometry(mainWidth * 0.5, 0, mainWidth * 0.5, mainHeight * 0.5);
    m_arducamView->setMaximumWidth(mainWidth * 0.5);
    m_arducamView->setMaximumHeight(mainHeight * 0.5);

    m_arducamScene = new QGraphicsScene(this);
    m_arducamPixmapItem = new QGraphicsPixmapItem();
    m_arducamScene->addItem(m_arducamPixmapItem);
    m_arducamView->setScene(m_arducamScene);

    m_microCam1View = new ZoomableGraphicsView("Micro Cam1 Output", this);
    m_microCam1View->setZoomLimits(0.05, 10.0);
    m_microCam1View->setFrameStyle(QFrame::Box);
    m_microCam1View->setGeometry(0, mainHeight * 0.5, mainWidth * 0.5, mainHeight * 0.5);
    m_microCam1View->setMaximumWidth(mainWidth * 0.5);
    m_microCam1View->setMaximumHeight(mainHeight * 0.5);

    m_microCam1Scene = new QGraphicsScene(this);
    m_microCam1PixmapItem = new QGraphicsPixmapItem();
    m_microCam1Scene->addItem(m_microCam1PixmapItem);
    m_microCam1View->setScene(m_microCam1Scene);

    m_microCam2View = new ZoomableGraphicsView("Micro Cam2 Output", this);
    m_microCam2View->setZoomLimits(0.05, 10.0);
    m_microCam2View->setFrameStyle(QFrame::Box);
    m_microCam2View->setGeometry(mainWidth * 0.5, mainHeight * 0.5, mainWidth * 0.5, mainHeight * 0.5);
    m_microCam2View->setMaximumWidth(mainWidth * 0.5);
    m_microCam2View->setMaximumHeight(mainHeight * 0.5);

    m_microCam2Scene = new QGraphicsScene(this);
    m_microCam2PixmapItem = new QGraphicsPixmapItem();
    m_microCam2Scene->addItem(m_microCam2PixmapItem);
    m_microCam2View->setScene(m_microCam2Scene);

    QGridLayout* mainLayout = new QGridLayout();
    mainLayout->addWidget(m_controlGroup, 0, 0);
    mainLayout->addWidget(m_arducamView, 0, 1);
    mainLayout->addWidget(m_microCam1View, 1, 0);
    mainLayout->addWidget(m_microCam2View, 1, 1);

    centralWidget->setLayout(mainLayout);

	// FPS dialog setup
    QDialog* FpsDialog = new QDialog(this);
    FpsDialog->setWindowTitle("FPS Monitor");
    FpsDialog->setFixedSize(150, 100);

    m_uiFPS = new QLabel("UI FPS: 0", FpsDialog);
    m_uiFPS->setAlignment(Qt::AlignRight);

    m_arducamFPS = new QLabel("Arducam FPS: 0", FpsDialog);
    m_arducamFPS->setAlignment(Qt::AlignRight);

    m_microCam1FPS = new QLabel("micro cam1 FPS: 0", FpsDialog);
    m_microCam1FPS->setAlignment(Qt::AlignRight);

    m_microCam2FPS = new QLabel("micro cam1 FPS: 0", FpsDialog);
    m_microCam2FPS->setAlignment(Qt::AlignRight);

    QVBoxLayout* fpsLayout = new QVBoxLayout(FpsDialog);
    fpsLayout->addWidget(m_uiFPS);
    fpsLayout->addWidget(m_arducamFPS);
    fpsLayout->addWidget(m_microCam1FPS);
    fpsLayout->addWidget(m_microCam2FPS);
    FpsDialog->setLayout(fpsLayout);
    FpsDialog->move(QPoint(FpsDialog->width() + mainWidth, FpsDialog->height()));

    // show based on flag. adding this check to all places is cumbersome. So, the flag decides visibility of the dialog instead of toggling the whole FPS logic.
    if (get_fpsDebug_flag()) FpsDialog->show();

    m_positionUpdateTimer = new QTimer(this);
    connect(m_positionUpdateTimer, &QTimer::timeout, this, &MainWindow::updatePositionDisplay);
    m_positionUpdateTimer->start(100); // Update every 100ms (0.1 second)
    m_prevX = globle_vars.current_x;
    m_prevY = globle_vars.current_y;
    m_prevZ = globle_vars.current_z;

    setupTransformationMatrix();

    // UI timer setup
    QTimer* uiUpdateTimer = new QTimer(this);
    connect(uiUpdateTimer, &QTimer::timeout, this, &MainWindow::renderLatestFrame);
    uiUpdateTimer->start(10); // making this faster updates the ui faster

	m_UITimer.start();

}


MainWindow::~MainWindow() {
    // Stop and cleanup timer
    if (m_positionUpdateTimer) {
        m_positionUpdateTimer->stop();
    }

    if (m_arducamOp.camWorker) {
        m_arducamOp.camWorker->stop();
    }
    if (m_arducamOp.thrd) {
        m_arducamOp.thrd->quit();
        m_arducamOp.thrd->wait();
    }

    if (m_macroImgInference.thrd) {
        m_macroImgInference.thrd->quit();
        m_macroImgInference.thrd->wait();
    }
    
    if (m_microCam1Op.camWorker) {
        m_microCam1Op.camWorker->stop();
    }
    if (m_microCam1Op.thrd) {
        m_microCam1Op.thrd->quit();
        m_microCam1Op.thrd->wait();
    }

    if (m_microCam2Op.camWorker) {
        m_microCam2Op.camWorker->stop();
    }
    if (m_microCam2Op.thrd) {
        m_microCam2Op.thrd->quit();
        m_microCam2Op.thrd->wait();
    }

    if (m_traverserThread) {
        m_traverser->abortTraversal();
        m_traverserThread->quit();
        m_traverserThread->wait();
    }
    //delete m_traverser;
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);

    int mainWidth = this->width();
    int mainHeight = this->height();

    m_controlGroup->setGeometry(0, 0, mainWidth * 0.5, mainHeight * 0.5);
    m_controlGroup->setMaximumWidth(mainWidth * 0.5);
    m_controlGroup->setMaximumHeight(mainHeight * 0.5);

    m_arducamView->setGeometry(mainWidth * 0.5, 0, mainWidth * 0.5, mainHeight * 0.5);
    m_arducamView->setMaximumWidth(mainWidth * 0.5);
    m_arducamView->setMaximumHeight(mainHeight * 0.5);

    m_microCam1View->setGeometry(0, mainHeight * 0.5, mainWidth * 0.5, mainHeight * 0.5);
    m_microCam1View->setMaximumWidth(mainWidth * 0.5);
    m_microCam1View->setMaximumHeight(mainHeight * 0.5);

    m_microCam2View->setGeometry(mainWidth * 0.5, mainHeight * 0.5, mainWidth * 0.5, mainHeight * 0.5);
    m_microCam2View->setMaximumWidth(mainWidth * 0.5);
    m_microCam2View->setMaximumHeight(mainHeight * 0.5);
}


QGroupBox* MainWindow::setupMovementUI() {
    // Movement controls
    QGridLayout* movementLayout = new QGridLayout();

    m_leftFastBtn = new QPushButton("←");
    m_leftSlowBtn = new QPushButton("←");
    m_leftSlowBtn->setStyleSheet("color: gray;");

    m_rightFastBtn = new QPushButton("→"); 
    m_rightSlowBtn = new QPushButton("→");
    m_rightSlowBtn->setStyleSheet("color: gray;");

    m_upFastBtn = new QPushButton("↑");
    m_upSlowBtn = new QPushButton("↑");
    m_upSlowBtn->setStyleSheet("color: gray;");

    m_downFastBtn = new QPushButton("↓");
    m_downSlowBtn = new QPushButton("↓");
    m_downSlowBtn->setStyleSheet("color: gray;");

    m_zUpBtn = new QPushButton("^");
    m_zUpFastBtn = new QPushButton("^^");
    m_zDownBtn = new QPushButton("v");
    m_zDownFastBtn = new QPushButton("vv");

    m_slant1Btn = new QPushButton("↖");
    m_slant2Btn = new QPushButton("↗");
    m_slant3Btn = new QPushButton("↘");
    m_slant4Btn = new QPushButton("↙");
    m_abortPathBtn = new QPushButton("⏻");
    m_resumePathBtn = new QPushButton("▶");
    m_resumePathBtn->hide();

    m_confirmAdjustmentBtn = new QPushButton("✔ Confirm");
    m_confirmAdjustmentBtn->setEnabled(false);

    m_leftFastBtn->setFixedSize(30, 30);
    m_leftSlowBtn->setFixedSize(30, 30);
    m_rightFastBtn->setFixedSize(30, 30);
    m_rightSlowBtn->setFixedSize(30, 30);
    m_upFastBtn->setFixedSize(30, 30);
    m_upSlowBtn->setFixedSize(30, 30);
    m_downFastBtn->setFixedSize(30, 30);
    m_downSlowBtn->setFixedSize(30, 30);
    m_zUpBtn->setFixedSize(30, 30);
    m_zUpFastBtn->setFixedSize(30, 30);
    m_zDownBtn->setFixedSize(30, 30);
    m_zDownFastBtn->setFixedSize(30, 30);
    m_slant1Btn->setFixedSize(30, 30);
    m_slant2Btn->setFixedSize(30, 30);
    m_slant3Btn->setFixedSize(30, 30);
    m_slant4Btn->setFixedSize(30, 30);
    m_abortPathBtn->setFixedSize(30, 30);
    m_resumePathBtn->setFixedSize(30, 30);
    m_confirmAdjustmentBtn->setFixedSize(80, 30);

    movementLayout->addWidget(m_slant1Btn, 1, 1);
    movementLayout->addWidget(m_upFastBtn, 0, 2);
    movementLayout->addWidget(m_upSlowBtn, 1, 2);
    movementLayout->addWidget(m_slant2Btn, 1, 3);
    movementLayout->addWidget(m_leftFastBtn, 2, 0);
    movementLayout->addWidget(m_leftSlowBtn, 2, 1);
    movementLayout->addWidget(m_rightFastBtn, 2, 4);
    movementLayout->addWidget(m_rightSlowBtn, 2, 3);
    movementLayout->addWidget(m_slant4Btn, 3, 1);
    movementLayout->addWidget(m_downFastBtn, 4, 2);
    movementLayout->addWidget(m_downSlowBtn, 3, 2);
    movementLayout->addWidget(m_slant3Btn, 3, 3);
    movementLayout->addWidget(m_zUpFastBtn, 0, 5);
    movementLayout->addWidget(m_zUpBtn, 1, 5);
    movementLayout->addWidget(m_zDownBtn, 3, 5);
    movementLayout->addWidget(m_zDownFastBtn, 4, 5);
    movementLayout->addWidget(m_abortPathBtn, 5, 0);
    movementLayout->addWidget(m_resumePathBtn, 5, 1);
    movementLayout->addWidget(m_confirmAdjustmentBtn, 5, 2);

    // Connect movement buttons to slots
    connect(m_leftFastBtn, &QPushButton::clicked, this, &MainWindow::onLeftFastClicked);
    connect(m_leftSlowBtn, &QPushButton::clicked, this, &MainWindow::onLeftSlowClicked);
    connect(m_rightFastBtn, &QPushButton::clicked, this, &MainWindow::onRightFastClicked);
    connect(m_rightSlowBtn, &QPushButton::clicked, this, &MainWindow::onRightSlowClicked);
    connect(m_upFastBtn, &QPushButton::clicked, this, &MainWindow::onUpFastClicked);
    connect(m_upSlowBtn, &QPushButton::clicked, this, &MainWindow::onUpSlowClicked);
    connect(m_downFastBtn, &QPushButton::clicked, this, &MainWindow::onDownFastClicked);
    connect(m_downSlowBtn, &QPushButton::clicked, this, &MainWindow::onDownSlowClicked);
    connect(m_zUpBtn, &QPushButton::clicked, this, &MainWindow::onZUpClicked);
    connect(m_zUpFastBtn, &QPushButton::clicked, this, &MainWindow::onZUpFastClicked);
    connect(m_zDownBtn, &QPushButton::clicked, this, &MainWindow::onZDownClicked);
    connect(m_zDownFastBtn, &QPushButton::clicked, this, &MainWindow::onZDownFastClicked);
    connect(m_slant1Btn, &QPushButton::clicked, this, &MainWindow::onSlant1Clicked);
    connect(m_slant2Btn, &QPushButton::clicked, this, &MainWindow::onSlant2Clicked);
    connect(m_slant3Btn, &QPushButton::clicked, this, &MainWindow::onSlant3Clicked);
    connect(m_slant4Btn, &QPushButton::clicked, this, &MainWindow::onSlant4Clicked);
    connect(m_abortPathBtn, &QPushButton::clicked, this, &MainWindow::onAbortPathClicked);
    //connect(m_resumePathBtn, &QPushButton::clicked, this, &MainWindow::onResumePathClicked);
    connect(m_confirmAdjustmentBtn, &QPushButton::clicked, this, &MainWindow::onConfirmAdjustmentClicked);

    QGroupBox* movementBox = new QGroupBox();
    movementBox->setLayout(movementLayout);

    return movementBox;
}

void MainWindow::setMovementControlsEnabled(bool enabled) {
    m_leftFastBtn->setEnabled(enabled);
    m_leftSlowBtn->setEnabled(enabled);
    m_rightFastBtn->setEnabled(enabled);
    m_rightSlowBtn->setEnabled(enabled);
    m_upFastBtn->setEnabled(enabled);
    m_upSlowBtn->setEnabled(enabled);
    m_downFastBtn->setEnabled(enabled);
    m_downSlowBtn->setEnabled(enabled);
    m_zUpBtn->setEnabled(enabled);
    m_zUpFastBtn->setEnabled(enabled);
    m_zDownBtn->setEnabled(enabled);
    m_zDownFastBtn->setEnabled(enabled);
    m_slant1Btn->setEnabled(enabled);
    m_slant2Btn->setEnabled(enabled);
    m_slant3Btn->setEnabled(enabled);
    m_slant4Btn->setEnabled(enabled);
    m_goToPositionBtn->setEnabled(enabled);
}


QGroupBox* MainWindow::setupPositionUI() {
    QLabel* currentLabel = new QLabel("Current Position");
    m_xLabel = new QLabel(QString("X: %1").arg(globle_vars.current_x));
    m_yLabel = new QLabel(QString("Y: %1").arg(globle_vars.current_y));
    m_zLabel = new QLabel(QString("Z: %1").arg(globle_vars.current_z));

    QLabel* newPosLabel = new QLabel("New Position 1");
    m_x1 = new QLineEdit("59079");
    m_y1 = new QLineEdit("161148");
    m_z1 = new QLineEdit("-960");
    m_stepEdit = new QLineEdit("100");

    QVBoxLayout* positionLayout = new QVBoxLayout();
    positionLayout->addWidget(currentLabel);
    positionLayout->addWidget(m_xLabel);
    positionLayout->addWidget(m_yLabel);
    positionLayout->addWidget(m_zLabel);
    positionLayout->addSpacing(10);
    positionLayout->addWidget(newPosLabel);
    positionLayout->addWidget(m_x1);
    positionLayout->addWidget(m_y1);
    positionLayout->addWidget(m_z1);
    positionLayout->addWidget(new QLabel("Step:"));
    positionLayout->addWidget(m_stepEdit);

    QGroupBox* positionBox = new QGroupBox();
    positionBox->setLayout(positionLayout);

    return positionBox;
}


QGroupBox* MainWindow::setupControlUI() {
    QVBoxLayout* controlLayout = new QVBoxLayout();

    m_arducamOp.cameraBtn = new QPushButton("Start Camera");
    QPushButton* captureMacroImg = new QPushButton("Capture Macro Img");
    QPushButton* predictMacroImg = new QPushButton("Predict Macro pos");
    m_goToPositionBtn = new QPushButton("Go To Position 1");
    m_microCam1Op.cameraBtn = new QPushButton("Start Duo Camera");
    QPushButton* captureMicroImg = new QPushButton("Capture Micro Img");
    m_predictMicroImg = new QPushButton("Path");

    controlLayout->addWidget(m_arducamOp.cameraBtn);
    controlLayout->addWidget(captureMacroImg);
    controlLayout->addWidget(predictMacroImg);
    controlLayout->addWidget(m_goToPositionBtn);
    controlLayout->addWidget(m_microCam1Op.cameraBtn);
    controlLayout->addWidget(captureMicroImg);
    controlLayout->addWidget(m_predictMicroImg);

    connect(m_arducamOp.cameraBtn, &QPushButton::clicked, this, &MainWindow::onStartArducam);
    connect(captureMacroImg, &QPushButton::clicked, this, &MainWindow::onCaptureMacroImg);
    connect(predictMacroImg, &QPushButton::clicked, this, &MainWindow::onPredictMacroImg);
    connect(m_goToPositionBtn, &QPushButton::clicked, this, &MainWindow::onGoToPosition1);
    connect(m_microCam1Op.cameraBtn, &QPushButton::clicked, this, &MainWindow::onStartDuocam);
    connect(captureMicroImg, &QPushButton::clicked, this, &MainWindow::onCaptureMicroImg);
    connect(m_predictMicroImg, &QPushButton::clicked, this, &MainWindow::onPredictMicroImg);

    QGroupBox* controlBox = new QGroupBox();
    controlBox->setLayout(controlLayout);

    return controlBox;
}

// Method to calculate 2D transformation matrix from 3 corresponding points
cv::Mat MainWindow::calculateTransformationMatrix(const std::vector<cv::Point2f>& imagePoints,
    const std::vector<cv::Point2f>& realPoints) {
    if (imagePoints.size() != 3 || realPoints.size() != 3) {
        LOG_WARNING("Exactly 3 points required for transformation matrix calculation");
        return cv::Mat();
    }

    // Calculate affine transformation matrix using 3 point pairs
    cv::Mat transformMatrix = cv::getAffineTransform(imagePoints, realPoints);

    LOG_INFO("Transformation matrix calculated successfully");
    std::stringstream ss;
    ss << "Affine Matrix is: " << transformMatrix;
    LOG_INFO(ss.str());

    return transformMatrix;
}

void MainWindow::renderLatestFrame() {
	QMutexLocker locker(&m_frameMutex);

	m_uiFrameCount++;
    if (m_UITimer.elapsed() >= 1000) {
        QString fpsText = "UI FPS - " + QString::number(m_uiFrameCount);
        m_uiFPS->setText(fpsText);
        m_uiFrameCount = 0;
        m_UITimer.restart();
    }

    if (!m_latestArducamImage.isNull()) {
        m_arducamPixmapItem->setPixmap(QPixmap::fromImage(m_latestArducamImage));
    } else {
        m_arducamPixmapItem->setPixmap(QPixmap());
	}

    if (!m_latestMicroCam1Image.isNull()) {
        m_microCam1PixmapItem->setPixmap(QPixmap::fromImage(m_latestMicroCam1Image));
    } else {
        m_microCam1PixmapItem->setPixmap(QPixmap());
    }

    if (!m_latestMicroCam2Image.isNull()) {
        m_microCam2PixmapItem->setPixmap(QPixmap::fromImage(m_latestMicroCam2Image));
    } else {
        m_microCam2PixmapItem->setPixmap(QPixmap());
    }
}


void MainWindow::updateFrame(const QImage& img, int camType) {
	QMutexLocker locker(&m_frameMutex);
    switch (camType) {
    case ARDUCAM:
    {
        m_latestArducamImage = img;
        m_arducamOp.frameCount++;
        if (m_arducamOp.FPSTimer.elapsed() >= 1000) {
			QString fpsText = "arducam FPS - " + QString::number(m_arducamOp.frameCount);
            m_arducamFPS->setText(fpsText);
            m_arducamOp.frameCount = 0;
            m_arducamOp.FPSTimer.restart();
        }
        break;
    }

    case MICROCAM1: {
        m_latestMicroCam1Image = img;
        m_microCam1Op.frameCount++;
        if (m_microCam1Op.FPSTimer.elapsed() >= 1000) {
            QString fpsText = "microCam1 FPS - " + QString::number(m_microCam1Op.frameCount);
            m_microCam1FPS->setText(fpsText);
            m_microCam1Op.frameCount = 0;
            m_microCam1Op.FPSTimer.restart();
        }
        break;
    }

    case MICROCAM2: {
        m_latestMicroCam2Image = img;
        m_microCam2Op.frameCount++;
        if (m_microCam2Op.FPSTimer.elapsed() >= 1000) {
            QString fpsText = "microCam2 FPS - " + QString::number(m_microCam2Op.frameCount);
            m_microCam2FPS->setText(fpsText);
            m_microCam2Op.frameCount = 0;
            m_microCam2Op.FPSTimer.restart();
        }
        break;
    }

    default:
        LOG_WARNING("Unknown camera type received in updateFrame: " << camType);
        break;
    }
    
}

void MainWindow::updatePositionDisplay() {
    // Check if values have changed to avoid unnecessary updates
    if (m_prevX != globle_vars.current_x ||
        m_prevY != globle_vars.current_y ||
        m_prevZ != globle_vars.current_z) {

        // Update the labels with new values (2 decimal places)
        m_xLabel->setText(QString("X: %1").arg(globle_vars.current_x));
        m_yLabel->setText(QString("Y: %1").arg(globle_vars.current_y));
        m_zLabel->setText(QString("Z: %1").arg(globle_vars.current_z));

        // Update previous values
        m_prevX = globle_vars.current_x;
        m_prevY = globle_vars.current_y;
        m_prevZ = globle_vars.current_z;
    }
}



void MainWindow::onStartArducam() { 
    if (m_arducamOp.thrd) {
        // Already running - stop!
        LOG_INFO("stopping arducam");
        m_arducamOp.toggleCamera();
        {
            QMutexLocker locker(&m_frameMutex);
            m_latestArducamImage = QImage();
        }
        m_arducamView->resetTransform();
        m_arducamOp.cameraBtn->setText("Start Arducam");
		m_currentMacroImg.release();
		m_macroImgPath.clear();
		m_macroImgPath.shrink_to_fit();
		m_arducamFPS->setText("arducam FPS - 0");
        return;
    }

    LOG_INFO("starting arducam");

    m_arducamOp.thrd = new QThread(this);

    int camIndex = get_camDebug_flag() ? IMG : WEBCAM; // WEBCAM needs to be replaced with correct slot value


	m_arducamOp.camWorker = new CameraWorker(IMG, 0, 3840, 2160, 20);// camIndex is 0 for arducam, 1 for microcam1 and 2 for microcam2
    m_arducamOp.camWorker->moveToThread(m_arducamOp.thrd);

	m_arducamView->resetTransform();
    m_arducamView->scale((float)m_arducamView->width()/ m_arducamOp.camWorker->getFrameWidth(), (float)m_arducamView->height() / m_arducamOp.camWorker->getFrameHeight());
    connect(m_arducamOp.thrd, &QThread::started, m_arducamOp.camWorker, &CameraWorker::process); 
    connect(m_arducamOp.camWorker, &CameraWorker::frameReady, this, &MainWindow::updateFrame, Qt::QueuedConnection); 
    connect(m_arducamOp.thrd, &QThread::finished, m_arducamOp.camWorker, &QObject::deleteLater); 
    
    m_arducamOp.cameraBtn->setText("Stop Camera");

    m_arducamOp.FPSTimer.start();

    m_arducamOp.thrd->start();
}

void MainWindow::onStartDuocam() {
    if (m_microCam1Op.thrd || m_microCam2Op.thrd) {
        // Already running - stop!
        LOG_INFO("stopping Duo cams");
        m_microCam1Op.toggleCamera();
        m_microCam2Op.toggleCamera();
        {
            QMutexLocker locker(&m_frameMutex);
            m_latestMicroCam1Image = QImage();
            m_latestMicroCam2Image = QImage();
        }
        m_microCam1Op.cameraBtn->setText("Start Duo Cam");
        m_microCam1View->resetTransform();
        m_microCam2View->resetTransform();
        m_microCam1FPS->setText("microCam1 FPS - 0");
        m_microCam2FPS->setText("microCam2 FPS - 0");
        return;
    }

    m_microCam1Op.thrd = new QThread(this);
    m_microCam1Op.camWorker = new CameraWorker(1, 1, 1280, 720, 20);
    m_microCam1Op.camWorker->moveToThread(m_microCam1Op.thrd);

    m_microCam1View->scale((float)m_microCam1View->width() / m_microCam1Op.camWorker->getFrameWidth(), (float)m_microCam1View->height() / m_microCam1Op.camWorker->getFrameHeight());

    connect(m_microCam1Op.thrd, &QThread::started, m_microCam1Op.camWorker, &CameraWorker::process);
    connect(m_microCam1Op.camWorker, &CameraWorker::frameReady, this, &MainWindow::updateFrame, Qt::QueuedConnection);
    connect(m_microCam1Op.thrd, &QThread::finished, m_microCam1Op.camWorker, &QObject::deleteLater);

    m_microCam1Op.thrd->start();
    m_microCam1Op.FPSTimer.start();


    m_microCam2Op.thrd = new QThread(this);
    m_microCam2Op.camWorker = new CameraWorker(3, 2, 1280, 720, 20);
    m_microCam2Op.camWorker->moveToThread(m_microCam2Op.thrd);

	m_microCam2View->scale((float)m_microCam2View->width() / m_microCam2Op.camWorker->getFrameWidth(), (float)m_microCam2View->height() / m_microCam2Op.camWorker->getFrameHeight());

    connect(m_microCam2Op.thrd, &QThread::started, m_microCam2Op.camWorker, &CameraWorker::process);
    connect(m_microCam2Op.camWorker, &CameraWorker::frameReady, this, &MainWindow::updateFrame, Qt::QueuedConnection);
    connect(m_microCam2Op.thrd, &QThread::finished, m_microCam2Op.camWorker, &QObject::deleteLater);

    m_microCam2Op.thrd->start();
    m_microCam2Op.FPSTimer.start();

    m_microCam1Op.cameraBtn->setText("Stop Duo Camera");
}

void MainWindow::onCaptureMacroImg() {
    if (!m_arducamOp.thrd) {
		LOG_WARNING("Arducam thread is not running. Cannot capture image.");
        return;
    }

	// this will ask the camera to take high res image - merging multiple frames
	// set the curFrame of MainWindow to the captured frame here and process this later for inference. So that we have easier access to the captured frame from all classes
    
	// TODO: eventually this will be read directly from the camera

	// TODO: might want to stop/pause the camera worker before capturing the image and calling setCapturedFrame... maybe?? if done this way then display the scaled down image here 

	// LOG_INFO("Captured frame"); for later use

    m_arducamOp.camWorker->setCaptureImg(true);
	QThread::msleep(100); // waiting to capture the image
	m_currentMacroImg = m_arducamOp.camWorker->getCaturedFrame().clone();

    // crop the black portions out
    //m_currentMacroImg = cropInputImage(m_arducamOp.camWorker->getCaturedFrame().clone());


    // === Save to macro_img folder ===
    // 1. Create folder path inside the project directory
    QString folderPath = QDir(QCoreApplication::applicationDirPath()).filePath("macro_img");
    QDir dir;
    if (!dir.exists(folderPath)) {
        dir.mkpath(folderPath);
    }

    // 2. Create file name based on date and time
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString filePath = folderPath + "/" + timestamp + ".png";

    // 3. Save using OpenCV imwrite
    if (!m_currentMacroImg.empty()) {
        cv::imwrite(filePath.toStdString(), m_currentMacroImg);
        LOG_INFO("Macro image saved to: " + filePath.toStdString());
    }
    else {
        LOG_WARNING("Captured macro image is empty. Not saving.");
    }
    
    
}

void MainWindow::inferenceResult(const cv::Mat& frame, const std::vector<cv::Rect>& boxCentroids) {

    // TODO: decide how to handle the inference result - directly update here or pass to camera worker?
    //m_arducamOp.camWorker->clearCapturedFrame(); // remove the captured frame
    //m_arducamOp.camWorker->setCapturedFrame(frame); // add the frame with detection rectangles
    //m_arducamOp.camWorker->start();

    // TODO: cameraworker::stop exits from the process loop, so camera thrd is no longer active
    // updating the frame as above does not show the rendered img as thrd is is not running
    // might need to add wait method to CameraWorker to actually wait and stop can be used to exit thrd?

    // save the output frame to a file
    cv::imwrite("output.jpg", frame);

    LOG_INFO("Showing inference result");

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(3840, 2160));
    QImage qImage(resized.data, resized.cols, resized.rows, resized.step, QImage::Format_RGB888);
    updateFrame(qImage.copy(), ARDUCAM);
    // copy the boxCentroids to use them later to change the color of detected boxes once processed
    m_macroImgPath.clear();
    m_macroImgPath = boxCentroids;

    // Clean up inference worker and thread
    m_macroImgInference.free();
    m_arducamOp.toggleCamera();
    m_arducamOp.cameraBtn->setText("Restart Arducam");

}

}

void MainWindow::setupTransformationMatrix() {
    // Example calibration points - replace with your actual calibration data
    std::vector<cv::Point2f> imagePoints = {
        cv::Point2f(1651 , 1195),   // Replace with actual image coordinates // i
        cv::Point2f(1878, 1094),   // from your calibration process// center
        cv::Point2f(2159, 1241)//0.1
    };

    std::vector<cv::Point2f> realPoints = {
        cv::Point2f(56730, 27795), // Replace with actual real world coordinates
        cv::Point2f(62000, 25602), // corresponding to the image points above
        cv::Point2f(68534, 28840)
    };

    m_transformMatrix = calculateTransformationMatrix(imagePoints, realPoints);
    LOG_INFO("Affine Matrix is ", m_transformMatrix);

    if (!m_transformMatrix.empty()) {
        LOG_INFO("Transformation matrix initialized successfully");
    }
    else {
        LOG_INFO("Failed to initialize transformation matrix");
    }
}



void MainWindow::onPredictMacroImg() {
    if (!m_arducamOp.thrd || m_macroImgInference.thrd) {
		LOG_WARNING("Arducam thread is not running or inference is already in progress.");
        return;
    }

    if (!m_arducamOp.camWorker->getCaptureImg()) {
		LOG_WARNING("No captured frame to process. Please capture an image first.");
        return;
    }

	std::string modelPath = "deps/models/yolov11n_trainedv1.onnx";
    if (!std::filesystem::exists(modelPath)) {
		    LOG_CRITICAL("Model file does not exist: " << modelPath);
        return;
	}

    m_arducamOp.camWorker->stop();
    
    {
		LOG_INFO("Starting inference on captured macro image...");

        m_macroImgInference.thrd = new QThread(this);


        m_macroImgInference.infWorker = new InferenceWorker(m_currentMacroImg.cols, m_currentMacroImg.rows, m_currentMacroImg);
        m_macroImgInference.infWorker->moveToThread(m_macroImgInference.thrd);

        connect(m_macroImgInference.thrd, &QThread::started, m_macroImgInference.infWorker, &InferenceWorker::predict);
        connect(m_macroImgInference.infWorker, &InferenceWorker::frameProcessed, this, &MainWindow::inferenceResult);
        connect(m_macroImgInference.thrd, &QThread::finished, m_macroImgInference.infWorker, &QObject::deleteLater);

        m_macroImgInference.thrd->start();
    }
    //m_arducamOp.camWorker->start(); // show the updated captured frame...

}




//----------------------------------------------------------------------------------------------------------------

void MainWindow::onCaptureMicroImg() {
    // ====== MicroCam1 ======
    if (!m_microCam1Op.thrd) {
        LOG_WARNING("First microCam is not running. Cannot capture image.");
        return;
    }
    m_microCam1Op.camWorker->setCaptureImg(true);
    QThread::msleep(30);
    m_currentMicroImg1 = m_microCam1Op.camWorker->getCaturedFrame().clone();

    // Save MicroCam1 image
    if (!m_currentMicroImg1.empty()) {
        QString folderPath1 = QDir(QCoreApplication::applicationDirPath()).filePath("micro_img1");
        QDir dir1;
        if (!dir1.exists(folderPath1)) {
            dir1.mkpath(folderPath1);
        }
        QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz");
        QString filePath1 = folderPath1 + "/" + timestamp + "_cam1.png";
        cv::imwrite(filePath1.toStdString(), m_currentMicroImg1);
        LOG_INFO("MicroCam1 image saved to: " + filePath1.toStdString());
    }
    else {
        LOG_WARNING("MicroCam1 captured image is empty. Not saving.");
    }

    // ====== MicroCam2 ======
    if (!m_microCam2Op.thrd) {
        LOG_WARNING("Second microCam is not running. Cannot capture image.");
        return;
    }
    m_microCam2Op.camWorker->setCaptureImg(true);
    QThread::msleep(30);
    m_currentMicroImg2 = m_microCam2Op.camWorker->getCaturedFrame().clone();

    // Save MicroCam2 image
    if (!m_currentMicroImg2.empty()) {
        QString folderPath2 = QDir(QCoreApplication::applicationDirPath()).filePath("micro_img2");
        QDir dir2;
        if (!dir2.exists(folderPath2)) {
            dir2.mkpath(folderPath2);
        }
        QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz");
        QString filePath2 = folderPath2 + "/" + timestamp + "_cam2.png";
        cv::imwrite(filePath2.toStdString(), m_currentMicroImg2);
        LOG_INFO("MicroCam2 image saved to: " + filePath2.toStdString());
    }
    else {
        LOG_WARNING("MicroCam2 captured image is empty. Not saving.");
    }




}
void MainWindow::onPredictMicroImg() {
    if (m_transformMatrix.empty()) {
        LOG_WARNING("Transformation matrix not set. Please calculate transformation matrix first.");
        LOG_INFO("Use calculateTransformationMatrix() with 3 corresponding image and real coordinate points.");
        return;
    }

    if (m_macroImgPath.empty()) {
        LOG_WARNING("No detected objects in macro image path. Please capture and predict macro image first.");
        return;
    }

    LOG_INFO("Starting traversal of detected macro image path...");

    m_traverser = new DetectionTraverser(&m_xyzStage);

    m_traverserThread = new QThread(this);
    m_traverser->moveToThread(m_traverserThread);
    m_traverser->setTraversalData(m_macroImgPath, m_transformMatrix);

    // Connect signals from the worker to slots in the main window
    connect(m_traverserThread, &QThread::started, m_traverser, &DetectionTraverser::process);
    connect(m_traverser, &DetectionTraverser::traversalStarted, this, &MainWindow::onTraversalStarted);
    connect(m_traverser, &DetectionTraverser::waitingForUserAdjustment, this, &MainWindow::onWaitingForUser);
    connect(m_traverser, &DetectionTraverser::traversalFinished, this, &MainWindow::onTraversalFinished);

    // For cleanup
    connect(m_traverserThread, &QThread::finished, m_traverserThread, &QObject::deleteLater);

    m_traverserThread->start();
}

void MainWindow::onTraversalStarted() {
    LOG_INFO("UI received traversalStarted signal. Disabling controls.");
    setMovementControlsEnabled(false);
    m_goToPositionBtn->setEnabled(false);
    m_confirmAdjustmentBtn->setEnabled(false);
    // Also disable the "Path" button to prevent starting twice
	m_predictMicroImg->setEnabled(false);
}

void MainWindow::onWaitingForUser() {
    LOG_INFO("UI received waitingForUserAdjustment signal. Enabling adjustment controls.");
    // NOW enable controls for fine-tuning
    setMovementControlsEnabled(true);
    m_confirmAdjustmentBtn->setEnabled(true);
}

void MainWindow::onConfirmAdjustmentClicked() {
    LOG_INFO("User confirmed adjustment. Capturing images and proceeding.");
    
    // First, disable controls again so user can't move during capture/next move
    setMovementControlsEnabled(false);
    m_confirmAdjustmentBtn->setEnabled(false);

    // Capture the images
    onCaptureMicroImg();
    //LOG_INFO("move_and_wait: Move completed. Proceeding.");
    // Tell the traverser thread to wake up and continue
    // Use invokeMethod to ensure it's called in the context of the other thread
    QMetaObject::invokeMethod(m_traverser, "userConfirmedAdjustment", Qt::QueuedConnection);
}

void MainWindow::onTraversalFinished(const QString& message) {
    //LOG_INFO("UI received traversalFinished signal: " << message.toStdString());

    // Re-enable all controls
    setMovementControlsEnabled(true);
    m_goToPositionBtn->setEnabled(true);
    m_confirmAdjustmentBtn->setEnabled(false); // Disable until next pause

    // Clean up the thread
    if (m_traverserThread) {
        m_traverserThread->quit();
        m_traverserThread->wait(); // ensure it's finished
        m_traverserThread = nullptr;
    }

	m_predictMicroImg->setEnabled(true);
}

void MainWindow::onAbortPathClicked() {
    LOG_INFO("Abort button clicked.");
    if (m_traverser) {
        // Use invokeMethod for thread safety
        QMetaObject::invokeMethod(m_traverser, "abortTraversal", Qt::QueuedConnection);
    }
    // The onTraversalFinished slot will handle UI cleanup
    m_predictMicroImg->setEnabled(true);
}


void MainWindow::onGoToPosition1() {  
    LOG_INFO("Move to Input Position 1");  
    double x = m_x1->text().toDouble();  
    double y = m_y1->text().toDouble();  
    double z = m_z1->text().toDouble();  
    m_xyzStage.move(x - globle_vars.current_x, 0, 0);
    m_xyzStage.move(0, y - globle_vars.current_y, 0);
    m_xyzStage.move(0, 0, z - globle_vars.current_z);
}

// movement slots

void MainWindow::onLeftFastClicked() { 
    LOG_INFO("Move Left Fast");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(-10.0* stepValue, 0.0, 0.0);
}

void MainWindow::onLeftSlowClicked() { 
    LOG_INFO("Move Left Slow");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(-stepValue, 0.0, 0.0);
}

void MainWindow::onRightFastClicked() {
	LOG_INFO("Move Right Fast");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(10.0* stepValue , 0.0, 0.0);
}

void MainWindow::onRightSlowClicked() {
	LOG_INFO("Move Right Slow");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(stepValue, 0.0, 0.0);
}

void MainWindow::onUpFastClicked() {
	LOG_INFO("Move Up Fast");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, -10.0 * stepValue, 0.0);
}

void MainWindow::onUpSlowClicked() {
	LOG_INFO("Move Up Slow");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, -stepValue, 0.0);
}

void MainWindow::onDownFastClicked() {
	LOG_INFO("Move Down Fast");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, 10.0* stepValue, 0.0);
}

void MainWindow::onDownSlowClicked() {
	LOG_INFO("Move Down Slow");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, stepValue, 0.0);
}

void MainWindow::onZUpClicked() {
	LOG_INFO("Move Z Up");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, 0.0, stepValue);
}

void MainWindow::onZUpFastClicked() {
	LOG_INFO("Move Z Up Fast");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, 0.0, 10.0* stepValue);
}

void MainWindow::onZDownClicked() {
	LOG_INFO("Move Z Down");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, 0.0, -stepValue);
}

void MainWindow::onZDownFastClicked() {
	LOG_INFO("Move Z Down Fast");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, 0.0, -10.0* stepValue);
}

void MainWindow::onSlant1Clicked() {
	LOG_INFO("Move ↖");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(-10.0* stepValue, -10.0* stepValue, 0.0);
}

void MainWindow::onSlant2Clicked() {
	LOG_INFO("Move ↗");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(10.0* stepValue, 0.0, 0.0);
    m_xyzStage.move(0.0, -10.0* stepValue, 0.0);
}

void MainWindow::onSlant3Clicked() {
	LOG_INFO("Move ↘");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(10.0* stepValue, 10.0* stepValue, 0.0);
}

void MainWindow::onSlant4Clicked() {
	LOG_INFO("Move ↙");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(-10.0* stepValue, 0.0, 0.0);
    m_xyzStage.move(0.0, 10.0* stepValue, 0.0);
}

void MainWindow::onResumePathClicked() {
    this->pause = false;
    LOG_INFO("Traversal will resume shortly.");
}
