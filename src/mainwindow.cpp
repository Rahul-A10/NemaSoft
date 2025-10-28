#include "mainwindow.h"
#include "utils.h"
#include <QComboBox>
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
#include <QDockWidget>
#include <logger.h>

// or more specific includes:
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#pragma execution_character_set("utf-8")


MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {



    QWidget* centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    this->setMinimumWidth(1920);
    this->setMinimumHeight(1000);

    int mainWidth = this->width();
    int mainHeight = this->height();

    m_xyzStage.setLogCallback([this](const QString& message, const QString& level) {
        this->log(message, level);
        });
    
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
    connect(m_arducamView, &ZoomableGraphicsView::imageClicked,this, &MainWindow::onArducamClicked);

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
    connect(m_microCam1View, &ZoomableGraphicsView::imageClicked,this, &MainWindow::onMicroCam1Clicked);
    

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
    connect(m_microCam2View, &ZoomableGraphicsView::imageClicked,this, &MainWindow::onMicroCam2Clicked);

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
    m_confirmAdjustmentBtn = new QPushButton("✔");
    m_confirmAdjustmentBtn->setEnabled(false);
	m_homeBtn = new QPushButton("🏠");
	

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
    m_confirmAdjustmentBtn->setFixedSize(30, 30);
    m_homeBtn->setFixedSize(30, 30);

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
	movementLayout->addWidget(m_homeBtn, 5, 3);

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
	connect(m_homeBtn, &QPushButton::clicked, this, &MainWindow::onHomeClicked);       
    

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
    ///m_goToPositionBtn->setEnabled(enabled);
}


QGroupBox* MainWindow::setupPositionUI() {
    // Left column widgets
    QLabel* currentLabel = new QLabel("Current Position");
    m_xLabel = new QLabel(QString("X: %1").arg(globle_vars.current_x));
    m_yLabel = new QLabel(QString("Y: %1").arg(globle_vars.current_y));
    m_zLabel = new QLabel(QString("Z: %1").arg(globle_vars.current_z));
    QLabel* newPosLabel = new QLabel("New Position 1");
    m_x1 = new QLineEdit("59852");
    m_y1 = new QLineEdit("142500");
    m_z1 = new QLineEdit("0");
    m_stepEdit = new QLineEdit("400");

    // Macro data dropdown
    QLabel* macroLabel = new QLabel("Macro Data:");
    m_macroComboBox = new QComboBox();
    m_macroComboBox->addItem("c.elegans", 1);
    m_macroComboBox->addItem("clump", 2);

    // Micro data dropdown
    QLabel* microLabel = new QLabel("Micro Data:");
    m_microComboBox = new QComboBox();
    m_microComboBox->addItem("Target", 1);
    m_microComboBox->addItem("Head", 2);
    m_microComboBox->addItem("Tail", 3);

    // Left column layout
    QVBoxLayout* leftColumnLayout = new QVBoxLayout();
    leftColumnLayout->addWidget(currentLabel);
    leftColumnLayout->addWidget(m_xLabel);
    leftColumnLayout->addWidget(m_yLabel);
    leftColumnLayout->addWidget(m_zLabel);
    leftColumnLayout->addSpacing(10);
    leftColumnLayout->addWidget(macroLabel);
    leftColumnLayout->addWidget(m_macroComboBox);
    leftColumnLayout->addWidget(microLabel);
    leftColumnLayout->addWidget(m_microComboBox);
    leftColumnLayout->addSpacing(10);
    leftColumnLayout->addWidget(newPosLabel);
    leftColumnLayout->addWidget(m_x1);
    leftColumnLayout->addWidget(m_y1);
    leftColumnLayout->addWidget(m_z1);
    leftColumnLayout->addWidget(new QLabel("Step:"));
    leftColumnLayout->addWidget(m_stepEdit);
    leftColumnLayout->addStretch(); // Push everything to the top

    // Right column - Log window
    QLabel* logLabel = new QLabel("Logs:");
    m_logDisplay = new LogDisplay(this);  // Use the new LogDisplay widget

    // Right column layout
    QVBoxLayout* rightColumnLayout = new QVBoxLayout();
    rightColumnLayout->addWidget(logLabel);
    rightColumnLayout->addWidget(m_logDisplay);  // Add the LogDisplay directly

    // Horizontal layout to combine both columns
    QHBoxLayout* mainLayout = new QHBoxLayout();
    mainLayout->addLayout(leftColumnLayout);
    mainLayout->addLayout(rightColumnLayout);

    // Optional: Set column width ratios (40% left, 60% right)
    mainLayout->setStretch(0, 2);  // Left column
    mainLayout->setStretch(1, 3);  // Right column (logs)

    QGroupBox* positionBox = new QGroupBox();
    positionBox->setLayout(mainLayout);
    return positionBox;
}


QGroupBox* MainWindow::setupControlUI() {
    // Test logging
    Logger::info(QString("Log Started"));

    QVBoxLayout* controlLayout = new QVBoxLayout();

    m_arducamOp.cameraBtn = new QPushButton("Start Camera");
    QPushButton* captureMacroImg = new QPushButton("Capture Macro Img");
    QPushButton* predictMacroImg = new QPushButton("Calculate Path");
	QPushButton* captureMacroData = new QPushButton("Capture Macro Data");
    m_goToPositionBtn = new QPushButton("Go To Position 1", this);
    m_microCam1Op.cameraBtn = new QPushButton("Start Duo Camera");
    QPushButton* captureMicroImg = new QPushButton("Capture Micro Img");
    QPushButton* m_trivarsePath = new QPushButton("Trivarse Path");
    m_predictMicroImg = new QPushButton("Predict Micro pos");
	QPushButton* captureMicroData = new QPushButton("Capture Micro Data");

    controlLayout->addWidget(m_arducamOp.cameraBtn);
    controlLayout->addWidget(captureMacroImg);
    controlLayout->addWidget(predictMacroImg);
	controlLayout->addWidget(captureMacroData);
    controlLayout->addWidget(m_goToPositionBtn);
    controlLayout->addWidget(m_microCam1Op.cameraBtn);
    controlLayout->addWidget(captureMicroImg);
	controlLayout->addWidget(m_trivarsePath);
    controlLayout->addWidget(m_predictMicroImg);
	controlLayout->addWidget(captureMicroData);

    connect(m_arducamOp.cameraBtn, &QPushButton::clicked, this, &MainWindow::onStartArducam);
    connect(captureMacroImg, &QPushButton::clicked, this, &MainWindow::onCaptureMacroImg);
    connect(predictMacroImg, &QPushButton::clicked, this, &MainWindow::onPredictMacroImg);
	connect(captureMacroData, &QPushButton::clicked, this, &MainWindow::onCaptureMacroData);
    connect(m_goToPositionBtn, &QPushButton::clicked, this, &MainWindow::onGoToPosition1);
    connect(m_microCam1Op.cameraBtn, &QPushButton::clicked, this, &MainWindow::onStartDuocam);
    connect(captureMicroImg, &QPushButton::clicked, this, &MainWindow::onCaptureMicroImg);
    connect(m_trivarsePath, &QPushButton::clicked, this, &MainWindow::onTriversePath);
    connect(m_predictMicroImg, &QPushButton::clicked, this, &MainWindow::onPredictMicroImg);
	connect(captureMicroData, &QPushButton::clicked, this, &MainWindow::onCaptureMicroData);

    QGroupBox* controlBox = new QGroupBox();
    controlBox->setLayout(controlLayout);

    return controlBox;
}

// Method to calculate 2D transformation matrix from 3 corresponding points
cv::Mat MainWindow::calculateTransformationMatrix(const std::vector<cv::Point2f>& imagePoints,
    const std::vector<cv::Point2f>& realPoints) {
    if (imagePoints.size() != 3 || realPoints.size() != 3) {
        log("Exactly 3 points required for transformation matrix calculation", "WARNING");
        return cv::Mat();
    }

    // Calculate affine transformation matrix using 3 point pairs
    cv::Mat transformMatrix = cv::getAffineTransform(imagePoints, realPoints);

    //log("Transformation matrix calculated successfully", "INFO");
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
        log(QString("Unknown camera type received in updateFrame: %1").arg(camType), "WARNING");
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
    clearMacroAnnotations();
    if (m_arducamOp.thrd) {
        // Already running - stop!
        log("stopping arducam", "INFO");
        m_arducamOp.toggleCamera();
        {
            QMutexLocker locker(&m_frameMutex);
            m_latestArducamImage = QImage();
        }
        m_arducamView->resetTransform();
        m_arducamOp.cameraBtn->setText("Start Arducam");
		m_currentMacroImg.release();
		m_currentMacroImgdata.release();
		m_macroImgPath.clear();
		m_macroImgPath.shrink_to_fit();
		m_arducamFPS->setText("arducam FPS - 0");
        m_macroAnnotations.clear();
        // Display the captured image
        updateMacroImageDisplay();
        return;
    }

    log("starting arducam","INFO");

    m_arducamOp.thrd = new QThread(this);

    int camIndex = get_camDebug_flag() ? IMG : WEBCAM; // WEBCAM needs to be replaced with correct slot value


	m_arducamOp.camWorker = new CameraWorker(IMG, 0, 3840, 2160, 20);// camIndex is 0 for arducam, 1 for microcam1 and 2 for microcam2
    m_arducamOp.camWorker->moveToThread(m_arducamOp.thrd);

	m_arducamView->resetTransform();
    m_arducamView->scale((float)m_arducamView->width()/ m_arducamOp.camWorker->getFrameWidth(), (float)m_arducamView->height() / m_arducamOp.camWorker->getFrameHeight());
    connect(m_arducamOp.thrd, &QThread::started, m_arducamOp.camWorker, &CameraWorker::process); 
    connect(m_arducamOp.camWorker, &CameraWorker::frameReady, this, &MainWindow::updateFrame, Qt::QueuedConnection); 
    connect(m_arducamOp.thrd, &QThread::finished, m_arducamOp.camWorker, &QObject::deleteLater); 
    clearMacroAnnotations();
    m_arducamOp.cameraBtn->setText("Stop Camera");

    m_arducamOp.FPSTimer.start();

    m_arducamOp.thrd->start();
}

void MainWindow::onStartDuocam() {
    if (m_microCam1Op.thrd || m_microCam2Op.thrd) {
        // Already running - stop!
        log("stopping Duo cams", "INFO");
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
    m_microCam1Op.camWorker = new CameraWorker(IMG, 1, 2720, 1536, 15);
    m_microCam1Op.camWorker->moveToThread(m_microCam1Op.thrd);

    m_microCam1View->scale((float)m_microCam1View->width() / m_microCam1Op.camWorker->getFrameWidth(), (float)m_microCam1View->height() / m_microCam1Op.camWorker->getFrameHeight());

    connect(m_microCam1Op.thrd, &QThread::started, m_microCam1Op.camWorker, &CameraWorker::process);
    connect(m_microCam1Op.camWorker, &CameraWorker::frameReady, this, &MainWindow::updateFrame, Qt::QueuedConnection);
    connect(m_microCam1Op.thrd, &QThread::finished, m_microCam1Op.camWorker, &QObject::deleteLater);

    m_microCam1Op.thrd->start();
    m_microCam1Op.FPSTimer.start();


    m_microCam2Op.thrd = new QThread(this);
    m_microCam2Op.camWorker = new CameraWorker(IMG, 2, 2720,1536, 15);
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
        log("Arducam thread is not running. Cannot capture image.","WARNING");
        return;
    }

    m_arducamOp.camWorker->setCaptureImg(true);
	QThread::msleep(300); // waiting to capture the image
	m_currentMacroImg = m_arducamOp.camWorker->getCaturedFrame().clone();
	m_currentMacroImgdata = m_currentMacroImg.clone();
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
        log(QString("Macro image saved to: %1").arg(filePath.toStdString()));
    }
    else {
        log("Captured macro image is empty. Not saving.", "WARNING");
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

    log("Showing inference result","INFO");

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(3840, 2160));
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
    QImage qImage(resized.data, resized.cols, resized.rows, resized.step, QImage::Format_RGB888);
    updateFrame(qImage.copy(), ARDUCAM);
    // copy the boxCentroids to use them later to change the color of detected boxes once processed
    m_macroImgPath = boxCentroids;
    // Clean up inference worker and thread
    m_macroImgInference.free();
    m_arducamOp.toggleCamera();

    m_arducamOp.cameraBtn->setText("Restart Arducam");

    if (m_macroImgInference.thrd) {
        m_macroImgInference.thrd->quit();
        m_macroImgInference.thrd->wait();
    }
}



void MainWindow::setupTransformationMatrix() {
    // Example calibration points - replace with your actual calibration data
    std::vector<cv::Point2f> imagePoints = {
        cv::Point2f(1445 , 1198),   // Replace with actual image coordinates // i
        cv::Point2f(1763, 1115),   // from your calibration process// center
        cv::Point2f(1865, 881)//0.1
    };

    std::vector<cv::Point2f> realPoints = {
        cv::Point2f(53272, 9943), // Replace with actual real world coordinates
        cv::Point2f(60295, 8102), // corresponding to the image points above
        cv::Point2f(62750, 2829)
    };

    m_transformMatrix = calculateTransformationMatrix(imagePoints, realPoints);

    QString matrixStr;
    if (!m_transformMatrix.empty()) {
        // Convert cv::Mat to string for logging
        std::ostringstream oss;
        oss << m_transformMatrix;
        matrixStr = QString::fromStdString(oss.str());
    }
    else {
        matrixStr = "Matrix is empty";
    }
    log(QString("Affine Matrix is: %1").arg(matrixStr), "INFO");

    if (!m_transformMatrix.empty()) {
        log("Transformation matrix initialized successfully","INFO");
    }
    else {
        log("Failed to initialize transformation matrix", "INFO");
    }
}



void MainWindow::onPredictMacroImg() {
    // Check if manual annotations already exist
    if (!m_macroAnnotations.isEmpty()) {
        log("Manual annotations detected. Using existing annotations for path traversal.", "INFO");

        // Filter annotations with class ID 1 and convert to cv::Rect
        std::vector<cv::Rect> manualBoxes;
        for (const YoloAnnotation& ann : m_macroAnnotations) {
            if (ann.classId == 1) {
                // Convert normalized coordinates to pixel coordinates
                int imageWidth = m_currentMacroImg.cols;
                int imageHeight = m_currentMacroImg.rows;

                float centerX = ann.center.x() * imageWidth;
                float centerY = ann.center.y() * imageHeight;
                float boxWidth = ann.width * imageWidth;
                float boxHeight = ann.height * imageHeight;

                // Create box with top-left corner coordinates
                int x = static_cast<int>(centerX - boxWidth / 2.0f);
                int y = static_cast<int>(centerY - boxHeight / 2.0f);
                int w = static_cast<int>(boxWidth);
                int h = static_cast<int>(boxHeight);

                // Clamp to image bounds
                x = max(0,min(x, imageWidth - w));
                y = max(0,min(y, imageHeight - h));
                w = max(1,min(w, imageWidth - x));
                h = max(1,min(h, imageHeight - y));

                cv::Rect box(x, y, w, h);
                manualBoxes.push_back(box);
            }
        }


        log(QString("Passing %1 manual annotations to InferenceWorker").arg(manualBoxes.size()), "INFO");

        // Create InferenceWorker with manual boxes
        m_macroImgInference.thrd = new QThread(this);
        m_macroImgInference.infWorker = new InferenceWorker(m_currentMacroImg.cols, m_currentMacroImg.rows, m_currentMacroImg, manualBoxes);
        m_macroImgInference.infWorker->moveToThread(m_macroImgInference.thrd);
        connect(m_macroImgInference.thrd, &QThread::started, m_macroImgInference.infWorker, &InferenceWorker::predictWithManualBoxes);
        connect(m_macroImgInference.infWorker, &InferenceWorker::frameProcessed, this, &MainWindow::inferenceResult);
        connect(m_macroImgInference.thrd, &QThread::finished, m_macroImgInference.infWorker, &QObject::deleteLater);
        m_macroImgInference.thrd->start();
        return;
    }

    if (!m_arducamOp.thrd || m_macroImgInference.thrd) {
        log("Arducam thread is not running or inference is already in progress.", "WARNING");
        return;
    }
    if (!m_arducamOp.camWorker->getCaptureImg()) {
        log("No captured frame to process. Please capture an image first.", "WARNING");
        return;
    }

    std::string modelPath = "deps/models/yolov11n_trainedv1.onnx";
    if (!std::filesystem::exists(modelPath)) {
        log(QString("Model file does not exist: %1").arg(QString::fromStdString(modelPath)), "CRITICAL");
        return;
    }

    m_arducamOp.camWorker->stop();

    {
        log("Starting inference on captured macro image...", "INFO");
        m_macroImgInference.thrd = new QThread(this);
        m_macroImgInference.infWorker = new InferenceWorker(m_currentMacroImg.cols, m_currentMacroImg.rows, m_currentMacroImg);
        m_macroImgInference.infWorker->moveToThread(m_macroImgInference.thrd);
        connect(m_macroImgInference.thrd, &QThread::started, m_macroImgInference.infWorker, &InferenceWorker::predict);
        connect(m_macroImgInference.infWorker, &InferenceWorker::frameProcessed, this, &MainWindow::inferenceResult);
        connect(m_macroImgInference.thrd, &QThread::finished, m_macroImgInference.infWorker, &QObject::deleteLater);
        m_macroImgInference.thrd->start();
    }
}

//----------------------------------------------DATA CAPTURE------------------------------------------------------------


void MainWindow::onArducamClicked(const QPointF& scenePos, const QPointF& imagePos) {
    log(QString("Arducam clicked - Scene:%1 Image:%2")
        .arg(QString::number(scenePos.x()) + "," + QString::number(scenePos.y()))
        .arg(QString::number(imagePos.x()) + "," + QString::number(imagePos.y())), "INFO");
    
    // Check if we have a captured macro image
    if (m_currentMacroImg.empty()) {
        log("No macro image captured yet", "INFO");
        return;
    }
    
    // Get image dimensions from the OpenCV Mat
    int imageWidth = m_currentMacroImg.cols;
    int imageHeight = m_currentMacroImg.rows;
    
    // Check if click is inside any existing bounding box (for deletion)
    for (int i = m_macroAnnotations.size() - 1; i >= 0; --i) {
        if (isClickInsideBox(imagePos, m_macroAnnotations[i], imageWidth, imageHeight)) {
            log(QString("Deleting annotation at index:%1").arg(i), "INFO");
            m_macroAnnotations.removeAt(i);
            updateMacroImageDisplay();
            return;
        }
    }
    
    // If not clicking on existing box, add new annotation
    // Use only class ID 0 for path traversing
    int classId = getSelectedMacroId();  // Force first class ID for path traversing
    
    // Normalize coordinates (YOLO format uses 0-1 range)
    double normalizedX = imagePos.x() / imageWidth;
    double normalizedY = imagePos.y() / imageHeight;
    
    // Default bounding box size (you can adjust these or make them configurable)
    double defaultWidth = 0.01;  // 1% of image width
    double defaultHeight = 0.01; // 1% of image height
    
    YoloAnnotation annotation;
    annotation.classId = classId;
    annotation.center = QPointF(normalizedX, normalizedY);
    annotation.width = defaultWidth;
    annotation.height = defaultHeight;
    
    m_macroAnnotations.append(annotation);
    
    log(QString("Manual annotation added - Class: %1, Center: (%2, %3), Size: (%4, %5)")
        .arg(classId)
        .arg(normalizedX)
        .arg(normalizedY)
        .arg(defaultWidth)
        .arg(defaultHeight), "INFO");
    
    // Update the display
    updateMacroImageDisplay();
}
bool MainWindow::isClickInsideBox(const QPointF& imagePos, const YoloAnnotation& ann, int imageWidth, int imageHeight) {
    // Convert normalized YOLO coordinates back to pixel coordinates
    double centerX = ann.center.x() * imageWidth;
    double centerY = ann.center.y() * imageHeight;
    double boxWidth = ann.width * imageWidth;
    double boxHeight = ann.height * imageHeight;

    // Calculate box boundaries
    double left = centerX - boxWidth / 2.0;
    double right = centerX + boxWidth / 2.0;
    double top = centerY - boxHeight / 2.0;
    double bottom = centerY + boxHeight / 2.0;

    // Check if click is inside
    return (imagePos.x() >= left && imagePos.x() <= right &&
        imagePos.y() >= top && imagePos.y() <= bottom);
}
void MainWindow::updateMacroImageDisplay() {
    if (m_currentMacroImg.empty()) {
        return;
    }

    // Clone the original image
    cv::Mat displayImg = m_currentMacroImg.clone();
    int imageWidth = displayImg.cols;
    int imageHeight = displayImg.rows;

    // Draw all annotation boxes
    for (int i = 0; i < m_macroAnnotations.size(); ++i) {
        const YoloAnnotation& ann = m_macroAnnotations[i];

        // Convert normalized coordinates to pixel coordinates
        double centerX = ann.center.x() * imageWidth;
        double centerY = ann.center.y() * imageHeight;
        double boxWidth = ann.width * imageWidth;
        double boxHeight = ann.height * imageHeight;

        // Calculate top-left corner
        int x = static_cast<int>(centerX - boxWidth / 2.0);
        int y = static_cast<int>(centerY - boxHeight / 2.0);
        int w = static_cast<int>(boxWidth);
        int h = static_cast<int>(boxHeight);

        // Get color for current class
        cv::Scalar color = getColorForClass(ann.classId);

        // Draw rectangle
        cv::rectangle(displayImg, cv::Rect(x, y, w, h), color, 2);

        // Find the class name in the combo box by searching for the matching ID
        QString className = "";
        for (int idx = 0; idx < m_macroComboBox->count(); ++idx) {
            if (m_macroComboBox->itemData(idx).toInt() == ann.classId) {
                className = m_macroComboBox->itemText(idx);
                break;
            }
        }

        // Draw class ID label
        std::string label = className.toStdString();
        cv::putText(displayImg, label, cv::Point(x, y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 1.5, color, 3);
    }

    // Convert to QImage
    cv::Mat rgbImg;
    cv::cvtColor(displayImg, rgbImg, cv::COLOR_BGR2RGB);
    QImage qImg(rgbImg.data, rgbImg.cols, rgbImg.rows, rgbImg.step, QImage::Format_RGB888);
    updateFrame(qImg.copy(), ARDUCAM);
}

/////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::onMicroCam1Clicked(const QPointF& scenePos, const QPointF& imagePos) {
    log(QString("MicroCam1 clicked - Scene: (%1, %2), Image: (%3, %4)").arg(scenePos.x()).arg(scenePos.y()).arg(imagePos.x()).arg(imagePos.y()), "INFO");
}



void MainWindow::onMicroCam2Clicked(const QPointF& scenePos, const QPointF& imagePos) {
    log(QString("MicroCam2 clicked - Scene: (%1, %2), Image: (%3, %4)").arg(scenePos.x()).arg(scenePos.y()).arg(imagePos.x()).arg(imagePos.y()), "INFO");
}

void MainWindow::clearMacroAnnotations() {
    m_macroAnnotations.clear();
    log("Cleared all macro annotations.", "INFO");
}
int MainWindow::getSelectedMacroId() const{
    int classId = m_macroComboBox->currentData().toInt();
    return classId;
}
QString MainWindow::getSelectedMacroLabel() const {
    return m_macroComboBox->currentText();
}
QString MainWindow::getSelectedMicroLabel() const {
    return m_microComboBox->currentText();
}

int MainWindow::getSelectedMicroId() const {
    return m_microComboBox->currentData().toInt();
}


void MainWindow::onCaptureMacroData() {
    QString timestamp;
    if (!m_currentMacroImgdata.empty()) {
        // Save Macro image
        QString folderPath = QDir(QCoreApplication::applicationDirPath()).filePath("macro_img_data/images");
        QDir dir;
        if (!dir.exists(folderPath)) {
            dir.mkpath(folderPath);
        }
        timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz");
        QString filePath = folderPath + "/" + timestamp + ".png";
        cv::imwrite(filePath.toStdString(), m_currentMacroImgdata);
        log("Macro image saved to: " + filePath, "INFO");

        // Save YOLO annotations
        QString labelsFolder = QDir(QCoreApplication::applicationDirPath()).filePath("macro_img_data/labels");
        if (!dir.exists(labelsFolder)) {
            dir.mkpath(labelsFolder);
        }
        QString labelFilePath = QDir(labelsFolder).filePath(timestamp + ".txt");

        // Write annotations in YOLO format
        QFile file(labelFilePath);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&file);

            // Write each annotation in YOLO format: class_id center_x center_y width height
            for (const YoloAnnotation& ann : m_macroAnnotations) {
                out << ann.classId << " "
                    << ann.center.x() << " "
                    << ann.center.y() << " "
                    << ann.width << " "
                    << ann.height << "\n";
            }

            file.close();
            log(QString("Saved %1 annotations to: %2").arg(m_macroAnnotations.size()).arg(labelFilePath), "INFO");
        }
        else {
            log("Failed to create label file: " + labelFilePath, "WARNING");
        }

        // Clear annotations after saving
    }
    else {
        log("Captured macro image is empty. Not saving.", "WARNING");
    }
}
void MainWindow::onCaptureMicroData() {
    if (!m_microCam1Op.thrd || !m_microCam2Op.thrd) {
        log("Duo cams are not running. Cannot capture data.", "WARNING");
        return;
    }
    m_microCam1Op.camWorker->setCaptureImg(true);
    QThread::msleep(50);
    m_currentMicroImg1 = m_microCam1Op.camWorker->getCaturedFrame().clone();
	m_microCam2Op.camWorker->setCaptureImg(true);
	QThread::msleep(50);
    m_currentMicroImg2 = m_microCam2Op.camWorker->getCaturedFrame().clone();
    QString timestamp;
    if (!m_currentMicroImg1.empty() && !m_currentMicroImg2.empty()) {
        // Save Macro image
        QString folderPath1 = QDir(QCoreApplication::applicationDirPath()).filePath("micro_img_data/cam1_images");
		QDir dir1;
        QString folderPath2 = QDir(QCoreApplication::applicationDirPath()).filePath("micro_img_data/cam2_images");
        QDir dir2;
        if (!dir1.exists(folderPath1)) {
			dir1.mkpath(folderPath1);//check and create the folder if it does not exist
        }
        if (!dir2.exists(folderPath2)) {
            dir2.mkpath(folderPath2);
		}


        timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz");
        QString filePath1 = folderPath1 + "/" +"cam1_" + timestamp + ".png";
        cv::imwrite(filePath1.toStdString(), m_currentMicroImg1);
		QString filePath2 = folderPath2 + "/" + "cam2_" + timestamp + ".png";
		cv::imwrite(filePath2.toStdString(), m_currentMicroImg2);

        log("Micro image saved to: " + filePath1, "INFO");

        // Save current stage position
        QString labelsFolder1 = QDir(QCoreApplication::applicationDirPath()).filePath("micro_img_data/cam1_labels");
        QDir ldir1;
        if (!ldir1.exists(labelsFolder1)) {
            ldir1.mkpath(labelsFolder1);
        }
        QString positionFilePath1 = QDir(labelsFolder1).filePath("cam1_" + timestamp + ".txt");
        // labels for cam 2
        QString labelsFolder2 = QDir(QCoreApplication::applicationDirPath()).filePath("micro_img_data/cam2_labels");
        QDir ldir2;
        if (!ldir2.exists(labelsFolder2)) {
            ldir2.mkpath(labelsFolder2);
        }
        QString positionFilePath2 = QDir(labelsFolder2).filePath("cam2_" + timestamp + ".txt");

        // Now write to the file
        QFile l_file1(positionFilePath1);
        if (l_file1.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&l_file1);
            // Write your position data here
            // out << positionData;
            l_file1.close();
            log("Position data saved to: " + positionFilePath1, "INFO");
        }
        else {
            log("Failed to create position file: " + positionFilePath1, "WARNING");
        }
		//repeat for cam 2
        QFile l_file2(positionFilePath2);
        if (l_file2.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&l_file2);
            // Write your position data here
            // out << positionData;
            l_file2.close();
            log("Position data saved to: " + positionFilePath2, "INFO");
        }
        else {
            log("Failed to create position file: " + positionFilePath2, "WARNING");;
        }
    }
    else {
        log("Captured macro image is empty. Not saving.", "WARNING");
    }
}

//----------------------------------------------------------------------------------------------------------------

void MainWindow::onCaptureMicroImg() {
    // ====== MicroCam1 ======
    if (!m_microCam1Op.thrd) {
        log("First microCam is not running. Cannot capture image.", "WARNING");
        return;
    }
    m_microCam1Op.camWorker->setCaptureImg(true);
    QThread::msleep(50);
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
        log("MicroCam1 image saved to: " + filePath1, "INFO");
		
    }
    else {
        log("MicroCam1 captured image is empty. Not saving.", "WARNING");
    }

    // ====== MicroCam2 ======
    if (!m_microCam2Op.thrd) {
        log("Second microCam is not running. Cannot capture image.", "WARNING");
        return;
    }
    m_microCam2Op.camWorker->setCaptureImg(true);
    QThread::msleep(50);
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
        log("MicroCam2 image saved to: " + filePath2, "INFO");
    }
    else {
        log("MicroCam2 captured image is empty. Not saving.", "WARNING");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(2000))); // wait for 300 ms to ensure images are saved properly before restarting the cams
    onStartDuocam(); // reset the micro cams after capturing the images
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(3000))); // wait for 300 ms to ensure images are saved properly before restarting the cams
	m_microCam1Op.cameraBtn->clicked(); // to update the button text to "Start Duo Cam"




}
void MainWindow::onPredictMicroImg() {
	log("Predict Micro Image button clicked.", "INFO");
}

void MainWindow::onTraversalStarted() {
    Logger::info("UI received traversalStarted signal. Disabling controls.");
    setMovementControlsEnabled(false);
    m_goToPositionBtn->setEnabled(false);
    m_confirmAdjustmentBtn->setEnabled(false);
    // Also disable the "Path" button to prevent starting twice
	m_predictMicroImg->setEnabled(false);
}

void MainWindow::onWaitingForUser() {
    Logger::info("UI received waitingForUserAdjustment signal. Enabling adjustment controls.");
    // NOW enable controls for fine-tuning
    setMovementControlsEnabled(true);
    m_confirmAdjustmentBtn->setEnabled(true);
}

void MainWindow::onConfirmAdjustmentClicked() {

    Logger::info("User confirmed adjustment. Capturing images and proceeding.");


    // First, disable controls again so user can't move during capture/next move
    setMovementControlsEnabled(false);
    m_confirmAdjustmentBtn->setEnabled(false);

    // Capture the images

    //onCaptureMicroImg();

    Logger::info("move_and_wait: Move completed. Proceeding.");
    // Tell the traverser thread to wake up and continue
	m_traverser->userConfirmedAdjustment();
}

void MainWindow::onHomeClicked() {
    m_xyzStage.home();
}

void MainWindow::onTraversalFinished(const QString& message) {
    Logger::info(QString("UI received traversalFinished signal: %1").arg(message));

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

void MainWindow::onTriversePath() {
        log("Path button clicked.", "INFO");
        if (m_transformMatrix.empty()) {
            log("Transformation matrix not set. Please calculate transformation matrix first.", "WARNING");
            log("Use calculateTransformationMatrix() with 3 corresponding image and real coordinate points.","INFO");
            return;
        }

        if (m_macroImgPath.empty()) {
            log("No detected objects in macro image path. Please capture and predict macro image first.", "WARNING");
            return;
        }

        /*if (m_xyzStage.getSerialHandle() == INVALID_HANDLE_VALUE){
            log("XYZ Stage not connected. Please connect the stage first.","WARNING" );
            return;
        }*/

        log("Starting traversal of detected macro image path...", "INFO");

        m_traverser = new DetectionTraverser(&m_xyzStage);
        m_traverserThread = new QThread(this);
        m_traverser->moveToThread(m_traverserThread);
        m_traverser->setTraversalData(m_macroImgPath, m_transformMatrix);


        

        // Connect signals from the worker to slots in the main window
        connect(m_traverserThread, &QThread::started, m_traverser, &DetectionTraverser::process);
        connect(m_traverser, &DetectionTraverser::logMessage, this, &MainWindow::log);
        connect(m_traverser, &DetectionTraverser::traversalStarted, this, &MainWindow::onTraversalStarted);
        connect(m_traverser, &DetectionTraverser::waitingForUserAdjustment, this, &MainWindow::onWaitingForUser);
        connect(m_traverser, &DetectionTraverser::traversalFinished, this, &MainWindow::onTraversalFinished);
        
        // For cleanup
        connect(m_traverserThread, &QThread::finished, m_traverserThread, &QObject::deleteLater);
        m_traverserThread->start();

}

void MainWindow::onAbortPathClicked() {
    log("Abort button clicked.", "INFO");
    if (m_traverser) {
        // Use invokeMethod for thread safety
        QMetaObject::invokeMethod(m_traverser, "abortTraversal", Qt::QueuedConnection);
    }
    // The onTraversalFinished slot will handle UI cleanup
    m_predictMicroImg->setEnabled(true);
}
void MainWindow::onGoToPosition1() {
    log("Move to Input Position 1", "INFO");
    double x = m_x1->text().toDouble();
    double y = m_y1->text().toDouble();
    double z = m_z1->text().toDouble();
    m_xyzStage.move(x - globle_vars.current_x, 0, 0);
    m_xyzStage.move(0, y - globle_vars.current_y, 0);
    m_xyzStage.move(0, 0, z - globle_vars.current_z);
}
// movement slots
void MainWindow::onLeftFastClicked() {
    log("Move Left Fast", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(-10.0 * stepValue, 0.0, 0.0);
}
void MainWindow::onLeftSlowClicked() {
    log("Move Left Slow", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(-stepValue, 0.0, 0.0);
}
void MainWindow::onRightFastClicked() {
    log("Move Right Fast", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(10.0 * stepValue, 0.0, 0.0);
}
void MainWindow::onRightSlowClicked() {
    log("Move Right Slow", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(stepValue, 0.0, 0.0);
}
void MainWindow::onUpFastClicked() {
    log("Move Up Fast", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, -10.0 * stepValue, 0.0);
}
void MainWindow::onUpSlowClicked() {
    log("Move Up Slow", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, -stepValue, 0.0);
}
void MainWindow::onDownFastClicked() {
    log("Move Down Fast", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, 10.0 * stepValue, 0.0);
}
void MainWindow::onDownSlowClicked() {
    log("Move Down Slow", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, stepValue, 0.0);
}
void MainWindow::onZUpClicked() {
    log("Move Z Up", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, 0.0, stepValue);
}
void MainWindow::onZUpFastClicked() {
    log("Move Z Up Fast", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, 0.0, 10.0 * stepValue);
}
void MainWindow::onZDownClicked() {
    log("Move Z Down", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, 0.0, -stepValue);
}
void MainWindow::onZDownFastClicked() {
    log("Move Z Down Fast", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(0.0, 0.0, -10.0 * stepValue);
}
void MainWindow::onSlant1Clicked() {
    log("Move ↖", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(-10.0 * stepValue, -10.0 * stepValue, 0.0);
}
void MainWindow::onSlant2Clicked() {
    log("Move ↗", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(10.0 * stepValue, 0.0, 0.0);
    m_xyzStage.move(0.0, -10.0 * stepValue, 0.0);
}
void MainWindow::onSlant3Clicked() {
    log("Move ↘", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(10.0 * stepValue, 10.0 * stepValue, 0.0);
}
void MainWindow::onSlant4Clicked() {
    log("Move ↙", "INFO");
    double stepValue = m_stepEdit->text().toDouble();
    m_xyzStage.move(-10.0 * stepValue, 0.0, 0.0);
    m_xyzStage.move(0.0, 10.0 * stepValue, 0.0);
}
void MainWindow::onResumePathClicked() {
    this->pause = false;
    log("Traversal will resume shortly.", "INFO");
}
void MainWindow::log(const QString& message, const QString& level) {
    try {
        if (level == "ERROR" || level == "CRITICAL") {
            Logger::error(message);
        }
        else if (level == "WARNING") {
            Logger::warning(message);
        }
        else if (level == "DEBUG") {
            Logger::debug(message);
        }
        else {
            Logger::info(message);
        }
    }
    catch (const std::exception& e) {
        // Handle standard exceptions
        qDebug() << "Logging failed with exception:" << e.what();
    }
    catch (...) {
        // Handle any other exceptions
        qDebug() << "Logging failed with an unknown exception.";
    }
}
