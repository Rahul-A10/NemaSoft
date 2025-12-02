#include "logger.h"
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
    if (!scene()) {
        event->ignore();
        return;
    }

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
    if (!scene()) {
        event->ignore();
        return;
    }

    if (event->button() == Qt::RightButton) {
        setDragMode(QGraphicsView::ScrollHandDrag);
    }

    if (event->button() == Qt::LeftButton) {
        // Get the position in view coordinates
        QPointF viewPos = event->pos();

        // Map to scene coordinates
        QPointF scenePos = mapToScene(viewPos.toPoint());

        // Get the pixmap item to find image coordinates
        QGraphicsPixmapItem* pixmapItem = nullptr;
        QList<QGraphicsItem*> items = scene()->items();
        for (QGraphicsItem* item : items) {
            pixmapItem = qgraphicsitem_cast<QGraphicsPixmapItem*>(item);
            if (pixmapItem) break;
        }

        if (pixmapItem) {
            // Map scene coordinates to item (image) coordinates
            QPointF imagePos = pixmapItem->mapFromScene(scenePos);

            // Check if click is within the image bounds
            QRectF imageRect = pixmapItem->boundingRect();
            if (imageRect.contains(imagePos)) {
                emit imageClicked(scenePos, imagePos);
            }
        }
    }
    QGraphicsView::mousePressEvent(event);
}

void ZoomableGraphicsView::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::RightButton) {
        setDragMode(QGraphicsView::RubberBandDrag);
    }
    QGraphicsView::mouseReleaseEvent(event);
}