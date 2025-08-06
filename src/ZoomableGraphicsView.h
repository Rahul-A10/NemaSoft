#ifndef ZOOMABLEGRAPHICSVIEW_H
#define ZOOMABLEGRAPHICSVIEW_H

#include <QGraphicsView>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QString>

class ZoomableGraphicsView : public QGraphicsView
{
    Q_OBJECT

public:
    explicit ZoomableGraphicsView(const QString& title, QWidget* parent = nullptr);

    QString getTitle() const;

    void setZoomLimits(double min, double max);
	//void setActivationStatus(bool activated) { m_activated = activated; }

protected:
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    QString m_title;
	 double m_maxScale = 10.0; // Maximum zoom level
	 double m_minScale = 0.1;  // Minimum zoom level to prevent excessive zoom out
	 bool m_activated = false; // Flag to track if the view is activated
};

#endif // ZOOMABLEGRAPHICSVIEW_H