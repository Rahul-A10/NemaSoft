

#include "ZoomableGraphicsView.h"
#include <QPainter>
#include <QFrame>
#include "utils.h"

ZoomableGraphicsView::ZoomableGraphicsView(const QString& title, QWidget* parent)
    : QGraphicsView(parent), m_title(title)
{
    setDragMode(QGraphicsView::RubberBandDrag);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorUnderMouse);

    // Optimize for frequent updates
    setViewportUpdateMode(QGraphicsView::MinimalViewportUpdate);
    setCacheMode(QGraphicsView::CacheBackground);

    // Add a border and title
    setFrameStyle(QFrame::Box);
    setLineWidth(2);
}

QString ZoomableGraphicsView::getTitle() const
{
    return m_title;
}

void ZoomableGraphicsView::setZoomLimits(double min, double max)
{
    m_minScale = min;
    m_maxScale = max;
}


void ZoomableGraphicsView::wheelEvent(QWheelEvent* event)
{
    const double scaleFactor = 1.15;
    // Get the current scale from the transformation matrix
    const double currentScale = transform().m11();
	// Limit the zoom level to prevent excessive scaling
    if (event->angleDelta().y() > 0) {
        if (currentScale < m_maxScale) {
            // Zoom in
            scale(scaleFactor, scaleFactor);
        }
    }
    else {
		if (currentScale > m_minScale) {
			// Zoom out
			scale(1.0 / scaleFactor, 1.0 / scaleFactor);
		}
    }
    event->accept();
}

void ZoomableGraphicsView::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        setDragMode(QGraphicsView::ScrollHandDrag);
    }
    QGraphicsView::mousePressEvent(event);
}

void ZoomableGraphicsView::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        setDragMode(QGraphicsView::RubberBandDrag);
    }
    QGraphicsView::mouseReleaseEvent(event);
}