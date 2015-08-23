/*
 * SlotGraphicsView.h
 *
 *  Created on: Jul 4, 2015
 *      Author: johan
 */

#ifndef PAP_GUI_SRC_SlotGraphicsView_H_
#define PAP_GUI_SRC_SlotGraphicsView_H_

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QMouseEvent>
#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QMouseEvent>
#include <QGraphicsSceneMouseEvent>
#include <ros/ros.h>
#include <QGraphicsRectItem>
#include <QGraphicsObject>

class SlotGraphicsScene: public QGraphicsScene {
Q_OBJECT
public:
	explicit SlotGraphicsScene(QWidget *parent = 0);

Q_SIGNALS:
	void sendMousePoint(int indexOfFiducial,QPointF point);
	void gotoPad(QPointF point);
	void dispensePad(QPointF point);

public Q_SLOTS:

void contextMenuEvent(QGraphicsSceneContextMenuEvent *event);

private:
};



class SlotGraphicsView: public QGraphicsView {
Q_OBJECT
public:
	explicit SlotGraphicsView(QWidget *parent = 0);
	SlotGraphicsView();
	virtual ~SlotGraphicsView();

Q_SIGNALS:


public Q_SLOTS:
	virtual void wheelEvent(QWheelEvent * event);

private:
};

#endif /* PAP_GUI_SRC_CLICKGRAPHICSVIEW_H_ */
