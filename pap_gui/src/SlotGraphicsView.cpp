/*
 * ClickGraphicsView.cpp
 *
 *  Created on: Jul 4, 2015
 *      Author: johan
 */

#include "../include/pap_gui/SlotGraphicsView.hpp"
#include "../include/pap_gui/main_window.hpp"
#include <QPointF>
#include <QDebug>
#include <QGraphicsRectItem>


// SCENE -------------------------------------------------------------------------------------------------------------
SlotGraphicsScene::SlotGraphicsScene(QWidget *parent) :
		QGraphicsScene(parent) {

}

void SlotGraphicsScene::contextMenuEvent(QGraphicsSceneContextMenuEvent *event) {
	QMenu menu;
	QPointF pt = event->scenePos();
	QAction *fiducial1 = new QAction("Set to current part", this);
	menu.addAction(fiducial1);

	QAction* selectedItem = menu.exec(event->screenPos());
	if (selectedItem) {
		if (selectedItem->text().toStdString() == "Set to current part") {
			Q_EMIT sendMousePoint(0, pt);
		}
	}
}


SlotGraphicsView::SlotGraphicsView() {
	// TODO Auto-generated constructor stub

}

SlotGraphicsView::~SlotGraphicsView() {
	// TODO Auto-generated destructor stub
}

SlotGraphicsView::SlotGraphicsView(QWidget *parent) :
QGraphicsView(parent)
{


}

void SlotGraphicsView::wheelEvent(QWheelEvent * event) {
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
