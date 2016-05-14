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

void graphicsScene::mousePressEvent(QGraphicsSceneMouseEvent* event){
    if(event->button() == Qt::LeftButton){
        x_press = event->scenePos().x();
        y_press = event->scenePos().y();

        temp_rect = new QGraphicsRectItem(x_press, y_press, 1, 1);
        temp_rect->setPen(QPen(Qt::red, 1, Qt::SolidLine));
        temp_rect->setBrush(Qt::red);

        this->addItem(temp_rect);
        update(QRectF(x_press, y_press, 1,1));
    }
}

void graphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent* event){

    width_press_ = event->scenePos().x() - x_press;
    height_press_ = event->scenePos().y() - y_press;

    temp_rect->setRect(x_press, y_press, width_press_ , height_press_);
    //temp_rect = new QGraphicsRectItem(x_press, y_press, width_press_ , height_press_);
    update(QRectF(x_press, y_press, width_press_, height_press_));
}

void graphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){

    QMessageBox msgBox;
    msgBox.setWindowTitle("New pad");
    msgBox.setText("Create new pad?");
    msgBox.setStandardButtons(QMessageBox::Yes);
    msgBox.addButton(QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::Yes);

    if (msgBox.exec() == QMessageBox::Yes) {
        std::cerr << "Creating new pad...\n";
        Q_EMIT createPad(QRectF(x_press, y_press, width_press_, height_press_));
    }else{
        this->removeItem(temp_rect);
        delete temp_rect;
        temp_rect = NULL;
    }

    update(QRectF(x_press, y_press, width_press_, height_press_));
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
