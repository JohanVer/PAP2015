/*
 * ClickGraphicsView.cpp
 *
 *  Created on: Jul 4, 2015
 *      Author: johan
 */

#include "../include/pap_gui/ClickGraphicsView.hpp"
#include "../include/pap_gui/main_window.hpp"
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
//scene = new QGraphicsScene();
//this->setSceneRect(50, 50, 350, 350);
//this->setScene(scene);

}

void ClickGraphicsView::mousePressEvent(QMouseEvent * e)
{
QPointF pt = mapToScene(e->pos());
Q_EMIT sendMousePoint (pt);
//delete (&pt);

}
void ClickGraphicsView::mouseMoveEvent (QMouseEvent *move)
{
//QPointF movment= mapToScene (move->pos ());
//delete (&movment);

}

void ClickGraphicsView::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu(this);
    QAction *fiducial = new QAction("Set as fiducial", this);
    menu.addAction(fiducial);
    QAction* selectedItem = menu.exec(event->globalPos());
    if(selectedItem){
    	if(selectedItem->text().toStdString() == "Set as fiducial" ){
    		Q_EMIT setFiducial(event->pos());
    	}
    }
}
