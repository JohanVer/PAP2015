/*
 * PadView.cpp
 *
 *  Created on: Jul 4, 2015
 *      Author: johan
 */

#include <pap_gui/PadView.hpp>
#include <pap_gui/main_window.hpp>
#include <QPointF>
#include <QDebug>
#include <QGraphicsRectItem>

graphicsScene::graphicsScene(QWidget *parent) :
    QGraphicsScene(parent) {

}

void graphicsScene::contextMenuEvent(QGraphicsSceneContextMenuEvent *event) {
    QMenu menu;
    QPointF pt = event->scenePos();
    QAction *fiducial1 = new QAction("Mark as 1. fiducial", this);
    QAction *fiducial2 = new QAction("Mark as 2. fiducial", this);
    QAction *gotoPoint = new QAction("Goto", this);
    QAction *dispensePadM = new QAction("Dispense Pad", this);
    QAction *exclude = new QAction("Delete", this);
    menu.addAction(fiducial1);
    menu.addAction(fiducial2);
    menu.addAction(gotoPoint);
    menu.addAction(dispensePadM);
    menu.addAction(exclude);
    QAction* selectedItem = menu.exec(event->screenPos());
    if (selectedItem) {
        if (selectedItem->text().toStdString() == "Mark as 1. fiducial") {
            Q_EMIT sendMousePoint(0, pt);
        } else if (selectedItem->text().toStdString()
                   == "Mark as 2. fiducial") {
            Q_EMIT sendMousePoint(1, pt);
        } else if (selectedItem->text().toStdString() == "Goto") {
            Q_EMIT gotoPad(pt);
        } else if (selectedItem->text().toStdString() == "Dispense Pad") {
            Q_EMIT dispensePad(pt);
        } else if (selectedItem->text().toStdString() == "Delete") {
            Q_EMIT deletePad(pt);
        }
    }
}

PadView::PadView() {
    // TODO Auto-generated constructor stub

}

PadView::~PadView() {
    // TODO Auto-generated destructor stub
}

PadView::PadView(QWidget *parent) :
    QGraphicsView(parent) {

}

void PadView::wheelEvent(QWheelEvent * event) {
    //if ctrl pressed, use original functionality
    if (event->modifiers() & Qt::ControlModifier) {
        QGraphicsView::wheelEvent(event);
    }

    else {
        if (event->delta() > 0) {
            scale(1.2, 1.2);
        } else {
            scale(0.8, 0.8);
        }
    }
}
