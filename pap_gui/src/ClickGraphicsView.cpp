/*
 * ClickGraphicsView.cpp
 *
 *  Created on: Jul 4, 2015
 *      Author: johan
 */

#include <pap_gui/ClickGraphicsView.hpp>
#include <pap_gui/main_window.hpp>
#include <QPointF>
#include <QDebug>

ClickGraphicsView::ClickGraphicsView() {
    // TODO Auto-generated constructor stub

}

ClickGraphicsView::~ClickGraphicsView() {
    // TODO Auto-generated destructor stub
}

ClickGraphicsView::ClickGraphicsView(QWidget *parent) :
    QGraphicsView(parent)
{


}

void ClickGraphicsView::wheelEvent(QWheelEvent * event) {
    //if ctrl pressed, use original functionality
    if (event->modifiers() & Qt::ControlModifier) {
        QGraphicsView::wheelEvent(event);
    }

    else {
        this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        if (event->delta() > 0) {
            this->scale(1.2, 1.2);
        } else {
            this->scale(0.8, 0.8);
        }
    }
}

void CameraGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent * e)
{
    QPointF pt = (e->scenePos());
    Q_EMIT sendMousePoint (pt);
}

CameraGraphicsScene::CameraGraphicsScene(QWidget *parent) :
    QGraphicsScene(parent) {

}

void CameraGraphicsScene::contextMenuEvent(QGraphicsSceneContextMenuEvent *event) {
    QMenu menu;
    QAction *fiducial = new QAction("Set as fiducial", this);
    menu.addAction(fiducial);
    QAction* selectedItem = menu.exec(event->screenPos());
    if(selectedItem){
        if(selectedItem->text().toStdString() == "Set as fiducial" ){
            Q_EMIT setFiducial(event->scenePos());
        }
    }
}
