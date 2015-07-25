/*
 * ClickGraphicsView.h
 *
 *  Created on: Jul 4, 2015
 *      Author: johan
 */

#ifndef PAP_GUI_SRC_PADVIEW_H_
#define PAP_GUI_SRC_PADVIEW_H_

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QMouseEvent>
#include <QGraphicsSceneMouseEvent>
#include <ros/ros.h>
#include <QGraphicsRectItem>
#include <QGraphicsObject>

class graphicsScene: public QGraphicsScene {
Q_OBJECT
public:
	explicit graphicsScene(QWidget *parent = 0);

Q_SIGNALS:
	void sendMousePoint(int indexOfFiducial,QPointF point);
	void gotoPad(QPointF point);
	void dispensePad(QPointF point);

public Q_SLOTS:

void contextMenuEvent(QGraphicsSceneContextMenuEvent *event);

private:
};


class PadView: public QGraphicsView {
Q_OBJECT
public:
	explicit PadView(QWidget *parent = 0);
	PadView();
	virtual ~PadView();

Q_SIGNALS:
	void sendMousePoint(QPointF point);
	void setFiducial(QPointF point);

public Q_SLOTS:
	virtual void wheelEvent(QWheelEvent * event);

private:
};

#endif /* PAP_GUI_SRC_PADVIEW_H_ */
