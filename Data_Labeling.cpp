#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QListWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QPixmap>
#include <QPainter>
#include <QMouseEvent>
#include <QStatusBar>
#include <QInputDialog>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <vector>
#include <string>

struct Annotation {
    int classIdx;
    QString className;
    double xCenter;
    double yCenter;
    QPoint canvasPos;
};

class ImageCanvas : public QLabel {
    Q_OBJECT

public:
    ImageCanvas(QWidget* parent = nullptr) : QLabel(parent) {
        setMinimumSize(800, 600);
        setStyleSheet("background-color: gray;");
        setAlignment(Qt::AlignCenter);
        setCursor(Qt::CrossCursor);
    }

    void setOriginalPixmap(const QPixmap& pixmap) {
        originalPixmap = pixmap;
        imageWidth = pixmap.width();
        imageHeight = pixmap.height();
        updateDisplay();
    }

    void updateDisplay() {
        if (originalPixmap.isNull()) return;

        QPixmap display = originalPixmap.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

        QPainter painter(&display);

        // Draw annotations
        QVector<QColor> colors = { Qt::red, Qt::blue, Qt::green, Qt::yellow,
                                  Qt::magenta, Qt::cyan, QColor(255, 165, 0) };

        for (const auto& ann : annotations) {
            QColor color = colors[ann.classIdx % colors.size()];
            painter.setPen(QPen(Qt::white, 2));
            painter.setBrush(color);

            QPoint scaledPos = scaleToDisplay(ann.canvasPos);
            painter.drawEllipse(scaledPos, 5, 5);

            painter.setPen(Qt::white);
            painter.setFont(QFont("Arial", 10, QFont::Bold));
            painter.drawText(scaledPos.x() - 20, scaledPos.y() - 10, ann.className);
        }

        setPixmap(display);
    }

    void addAnnotation(const Annotation& ann) {
        annotations.push_back(ann);
        updateDisplay();
    }

    void clearAnnotations() {
        annotations.clear();
        updateDisplay();
    }

    void removeLastAnnotation() {
        if (!annotations.empty()) {
            annotations.pop_back();
            updateDisplay();
        }
    }

    void removeAnnotation(int index) {
        if (index >= 0 && index < annotations.size()) {
            annotations.erase(annotations.begin() + index);
            updateDisplay();
        }
    }

    const std::vector<Annotation>& getAnnotations() const {
        return annotations;
    }

    int getImageWidth() const { return imageWidth; }
    int getImageHeight() const { return imageHeight; }

signals:
    void clicked(QPoint pos);

protected:
    void mousePressEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton && !originalPixmap.isNull()) {
            emit clicked(event->pos());
        }
    }

    void resizeEvent(QResizeEvent* event) override {
        QLabel::resizeEvent(event);
        updateDisplay();
    }

private:
    QPoint scaleToDisplay(const QPoint& originalPos) {
        if (originalPixmap.isNull()) return originalPos;

        QSize displaySize = originalPixmap.scaled(size(), Qt::KeepAspectRatio,
            Qt::SmoothTransformation).size();

        double scaleX = (double)displaySize.width() / imageWidth;
        double scaleY = (double)displaySize.height() / imageHeight;

        int offsetX = (width() - displaySize.width()) / 2;
        int offsetY = (height() - displaySize.height()) / 2;

        return QPoint(originalPos.x() * scaleX + offsetX,
            originalPos.y() * scaleY + offsetY);
    }

    QPixmap originalPixmap;
    std::vector<Annotation> annotations;
    int imageWidth = 0;
    int imageHeight = 0;
};

class YOLOLabelingTool : public QMainWindow {
    Q_OBJECT

public:
    YOLOLabelingTool(QWidget* parent = nullptr) : QMainWindow(parent) {
        setupUI();
        initializeClasses();
    }

private:
    void setupUI() {
        setWindowTitle("YOLO Data Labeling Tool - C++");
        setMinimumSize(1200, 800);

        QWidget* centralWidget = new QWidget(this);
        setCentralWidget(centralWidget);

        QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);

        // Left side - controls and canvas
        QVBoxLayout* leftLayout = new QVBoxLayout();

        // Control panel
        QHBoxLayout* controlLayout = new QHBoxLayout();

        QPushButton* loadBtn = new QPushButton("Load Image");
        connect(loadBtn, &QPushButton::clicked, this, &YOLOLabelingTool::loadImage);
        controlLayout->addWidget(loadBtn);

        QPushButton* setDirBtn = new QPushButton("Set Output Dir");
        connect(setDirBtn, &QPushButton::clicked, this, &YOLOLabelingTool::setOutputDir);
        controlLayout->addWidget(setDirBtn);

        controlLayout->addWidget(new QLabel("Class:"));

        classCombo = new QComboBox();
        connect(classCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &YOLOLabelingTool::onClassChanged);
        controlLayout->addWidget(classCombo);

        QPushButton* addClassBtn = new QPushButton("Add Class");
        connect(addClassBtn, &QPushButton::clicked, this, &YOLOLabelingTool::addCustomClass);
        controlLayout->addWidget(addClassBtn);

        QPushButton* undoBtn = new QPushButton("Undo");
        connect(undoBtn, &QPushButton::clicked, this, &YOLOLabelingTool::undoLast);
        controlLayout->addWidget(undoBtn);

        QPushButton* clearBtn = new QPushButton("Clear All");
        connect(clearBtn, &QPushButton::clicked, this, &YOLOLabelingTool::clearAll);
        controlLayout->addWidget(clearBtn);

        QPushButton* saveBtn = new QPushButton("Save & Next");
        saveBtn->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;");
        connect(saveBtn, &QPushButton::clicked, this, &YOLOLabelingTool::saveAndNext);
        controlLayout->addWidget(saveBtn);

        controlLayout->addStretch();
        leftLayout->addLayout(controlLayout);

        // Canvas
        canvas = new ImageCanvas();
        connect(canvas, &ImageCanvas::clicked, this, &YOLOLabelingTool::onCanvasClick);
        leftLayout->addWidget(canvas, 1);

        mainLayout->addLayout(leftLayout, 1);

        // Right side - annotations list
        QVBoxLayout* rightLayout = new QVBoxLayout();

        rightLayout->addWidget(new QLabel("<b>Annotations:</b>"));

        annotationsList = new QListWidget();
        rightLayout->addWidget(annotationsList, 1);

        QPushButton* deleteBtn = new QPushButton("Delete Selected");
        connect(deleteBtn, &QPushButton::clicked, this, &YOLOLabelingTool::deleteSelected);
        rightLayout->addWidget(deleteBtn);

        rightLayout->addWidget(new QLabel("<b>Classes:</b>"));

        classList = new QListWidget();
        classList->setMaximumHeight(150);
        rightLayout->addWidget(classList);

        QWidget* rightWidget = new QWidget();
        rightWidget->setLayout(rightLayout);
        rightWidget->setMaximumWidth(300);
        mainLayout->addWidget(rightWidget);

        // Status bar
        statusBar()->showMessage("Set output directory to begin");
    }

    void initializeClasses() {
        classNames = { "person", "car", "animal", "object", "landmark" };
        updateClassCombo();
        updateClassList();
        currentClassIdx = 0;
    }

    void updateClassCombo() {
        classCombo->clear();
        for (const auto& className : classNames) {
            classCombo->addItem(QString::fromStdString(className));
        }
    }

    void updateClassList() {
        classList->clear();
        for (size_t i = 0; i < classNames.size(); ++i) {
            classList->addItem(QString("%1: %2").arg(i).arg(QString::fromStdString(classNames[i])));
        }
    }

    void updateAnnotationsList() {
        annotationsList->clear();
        const auto& annotations = canvas->getAnnotations();
        for (size_t i = 0; i < annotations.size(); ++i) {
            annotationsList->addItem(QString("%1. %2 (%.3f, %.3f)")
                .arg(i + 1)
                .arg(annotations[i].className)
                .arg(annotations[i].xCenter)
                .arg(annotations[i].yCenter));
        }
    }

    void saveClassesFile() {
        if (outputDir.isEmpty()) return;

        QString classesPath = outputDir + QDir::separator() + "classes.txt";
        QFile file(classesPath);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&file);
            for (const auto& className : classNames) {
                out << QString::fromStdString(className) << "\n";
            }
            file.close();
        }
    }

private slots:
    void setOutputDir() {
        QString dir = QFileDialog::getExistingDirectory(this, "Select Output Directory");
        if (!dir.isEmpty()) {
            outputDir = QDir::toNativeSeparators(dir);
            imagesDir = outputDir + QDir::separator() + "images";
            labelsDir = outputDir + QDir::separator() + "labels";

            // Create directories
            QDir outputQDir(outputDir);
            if (!outputQDir.exists("images")) {
                outputQDir.mkdir("images");
            }
            if (!outputQDir.exists("labels")) {
                outputQDir.mkdir("labels");
            }

            saveClassesFile();

            statusBar()->showMessage("Output: " + outputDir);
            QMessageBox::information(this, "Success",
                QString("Output directories created:\n%1\n%2").arg(imagesDir).arg(labelsDir));
        }
    }

    void loadImage() {
        if (outputDir.isEmpty()) {
            QMessageBox::warning(this, "No Output Directory",
                "Please set output directory first!");
            return;
        }

        QString fileName = QFileDialog::getOpenFileName(this, "Select Image",
            "", "Image Files (*.jpg *.jpeg *.png *.bmp)");

        if (!fileName.isEmpty()) {
            currentImagePath = fileName;
            currentImageName = QFileInfo(fileName).fileName();

            QPixmap pixmap(fileName);
            canvas->setOriginalPixmap(pixmap);
            canvas->clearAnnotations();
            updateAnnotationsList();

            statusBar()->showMessage("Loaded: " + currentImageName);
        }
    }

    void onCanvasClick(QPoint pos) {
        if (canvas->getImageWidth() == 0) {
            QMessageBox::warning(this, "No Image", "Please load an image first!");
            return;
        }

        // Convert canvas coordinates to image coordinates
        QPixmap pix = canvas->pixmap();
        if (pix.isNull()) return;

        QSize displaySize = pix.size();
        int offsetX = (canvas->width() - displaySize.width()) / 2;
        int offsetY = (canvas->height() - displaySize.height()) / 2;

        int x = pos.x() - offsetX;
        int y = pos.y() - offsetY;

        if (x < 0 || y < 0 || x >= displaySize.width() || y >= displaySize.height()) {
            return;
        }

        double scaleX = (double)canvas->getImageWidth() / displaySize.width();
        double scaleY = (double)canvas->getImageHeight() / displaySize.height();

        int imgX = x * scaleX;
        int imgY = y * scaleY;

        // Normalize to YOLO format
        double xCenter = (double)imgX / canvas->getImageWidth();
        double yCenter = (double)imgY / canvas->getImageHeight();

        Annotation ann;
        ann.classIdx = currentClassIdx;
        ann.className = QString::fromStdString(classNames[currentClassIdx]);
        ann.xCenter = xCenter;
        ann.yCenter = yCenter;
        ann.canvasPos = QPoint(imgX, imgY);

        canvas->addAnnotation(ann);
        updateAnnotationsList();

        statusBar()->showMessage(QString("Added %1 at (%.3f, %.3f)")
            .arg(ann.className).arg(xCenter).arg(yCenter));
    }

    void onClassChanged(int index) {
        currentClassIdx = index;
        statusBar()->showMessage(QString("Current class: %1 (ID: %2)")
            .arg(QString::fromStdString(classNames[index])).arg(index));
    }

    void addCustomClass() {
        bool ok;
        QString className = QInputDialog::getText(this, "Add Class",
            "Enter new class name:", QLineEdit::Normal, "", &ok);

        if (ok && !className.isEmpty()) {
            classNames.push_back(className.toStdString());
            updateClassCombo();
            updateClassList();
            saveClassesFile();
            statusBar()->showMessage("Added class: " + className);
        }
    }

    void undoLast() {
        canvas->removeLastAnnotation();
        updateAnnotationsList();
        statusBar()->showMessage("Last annotation removed");
    }

    void clearAll() {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Clear All",
            "Remove all annotations?", QMessageBox::Yes | QMessageBox::No);

        if (reply == QMessageBox::Yes) {
            canvas->clearAnnotations();
            updateAnnotationsList();
            statusBar()->showMessage("All annotations cleared");
        }
    }

    void deleteSelected() {
        int index = annotationsList->currentRow();
        if (index >= 0) {
            canvas->removeAnnotation(index);
            updateAnnotationsList();
            statusBar()->showMessage(QString("Deleted annotation %1").arg(index + 1));
        }
    }

    void saveAndNext() {
        if (currentImagePath.isEmpty()) {
            QMessageBox::warning(this, "No Image", "No image loaded!");
            return;
        }

        if (outputDir.isEmpty()) {
            QMessageBox::warning(this, "No Output Directory",
                "Please set output directory first!");
            return;
        }

        // Copy image to images folder
        QString imageDest = imagesDir + QDir::separator() + currentImageName;

        // Remove destination file if it exists
        if (QFile::exists(imageDest)) {
            QFile::remove(imageDest);
        }

        if (!QFile::copy(currentImagePath, imageDest)) {
            QMessageBox::warning(this, "Error", "Failed to copy image!");
            return;
        }

        // Save annotations to labels folder
        QString baseName = QFileInfo(currentImageName).baseName();
        QString labelPath = labelsDir + QDir::separator() + baseName + ".txt";

        QFile file(labelPath);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&file);
            const auto& annotations = canvas->getAnnotations();

            for (const auto& ann : annotations) {
                // YOLO format: class_id x_center y_center width height
                out << QString("%1 %2 %3 0.01 0.01\n")
                    .arg(ann.classIdx)
                    .arg(ann.xCenter, 0, 'f', 6)
                    .arg(ann.yCenter, 0, 'f', 6);
            }
            file.close();
        }
        else {
            QMessageBox::warning(this, "Error", "Failed to save label file!");
            return;
        }

        statusBar()->showMessage(QString("Saved: %1 with %2 annotations")
            .arg(currentImageName).arg(canvas->getAnnotations().size()));

        QMessageBox::information(this, "Saved",
            QString("Image and labels saved!\nImage: %1\nLabel: %2")
            .arg(imageDest).arg(labelPath));

        // Clear for next image
        canvas->clearAnnotations();
        updateAnnotationsList();

        QMessageBox::StandardButton reply = QMessageBox::question(this,
            "Load Next", "Load another image?",
            QMessageBox::Yes | QMessageBox::No);

        if (reply == QMessageBox::Yes) {
            loadImage();
        }
    }

private:
    ImageCanvas* canvas;
    QComboBox* classCombo;
    QListWidget* annotationsList;
    QListWidget* classList;

    std::vector<std::string> classNames;
    int currentClassIdx = 0;

    QString outputDir;
    QString imagesDir;
    QString labelsDir;
    QString currentImagePath;
    QString currentImageName;
};

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    YOLOLabelingTool window;
    window.show();

    return app.exec();
}