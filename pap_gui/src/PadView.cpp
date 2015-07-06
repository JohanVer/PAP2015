/*
 * PadView.cpp
 *
 *  Created on: Jul 4, 2015
 *      Author: johan
 */

#include "../include/pap_gui/PadView.hpp"
#include "../include/pap_gui/main_window.hpp"
#include <QPointF>
#include <QDebug>
#include <QGraphicsRectItem>



graphicsScene::graphicsScene(QWidget *parent) :
		QGraphicsScene(parent) {

}
/*
void graphicsScene::contextMenuEvent(QGraphicsSceneContextMenuEvent *event) {
	QMenu menu;
	QAction *fiducial = new QAction("Mark", this);
	menu.addAction(fiducial);
	QAction* selectedItem = menu.exec(event->screenPos());
	if (selectedItem) {
		if (selectedItem->text().toStdString() == "Mark") {
			ROS_INFO("Pos: %f %f",event->scenePos().x(),event->scenePos().y());
			//Q_EMIT setFiducial(event->pos());
		}
	}
}
*/
/*
void graphicsScene::mousePressEvent(QGraphicsSceneMouseEvent * e) {
	QPointF pt = e->scenePos();
	ROS_INFO("Pos X : %f Y: %f",pt.x(),pt.y());
	//Q_EMIT sendMousePoint(pt);
//delete (&pt);

}
*/
PadView::PadView() {
	// TODO Auto-generated constructor stub

}

PadView::~PadView() {
	// TODO Auto-generated destructor stub
}

PadView::PadView(QWidget *parent) :
		QGraphicsView(parent) {
//scene = new QGraphicsScene();
//this->setSceneRect(50, 50, 350, 350);
//this->setScene(scene);

}
/*
void PadView::mousePressEvent(QMouseEvent * e) {
	QPointF pt = mapToScene(e->pos());
	Q_EMIT sendMousePoint(pt);
//delete (&pt);

}
*/
/*
void PadView::mouseMoveEvent(QMouseEvent *move) {
//QPointF movment= mapToScene (move->pos ());
//delete (&movment);

}
*/
/*
 void PadView::contextMenuEvent(QContextMenuEvent *event) {
 QMenu menu(this);
 QAction *fiducial = new QAction("Mark", this);
 menu.addAction(fiducial);
 QAction* selectedItem = menu.exec(event->globalPos());
 if (selectedItem) {
 if (selectedItem->text().toStdString() == "Mark") {
 Q_EMIT setFiducial(event->pos());
 }
 }
 }
 */
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
