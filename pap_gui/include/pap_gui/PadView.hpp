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

class graphicsScene: public QGraphicsScene {
Q_OBJECT
public:
	explicit graphicsScene(QWidget *parent = 0);

Q_SIGNALS:
	void sendMousePoint(QPointF point);

public Q_SLOTS:
	//void mousePressEvent(QMouseEvent * e);
//void mousePressEvent(QGraphicsSceneMouseEvent *e) {
	//ROS_INFO("Hallo1");
//}
//void contextMenuEvent(QGraphicsSceneContextMenuEvent *event);
	//void contextMenuEvent(QContextMenuEvent *event);
private:
};

class rectObject : public QGraphicsRectItem
{
public:
	explicit rectObject(qreal x, qreal y, qreal width, qreal height, QGraphicsItem * parent = 0):
	QGraphicsRectItem( x,  y,  width,  height, parent){
		id_ = 0;
	}

	int id_;

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event)
    {
        ROS_INFO("My id : %d",id_);
    }
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
	//void mousePressEvent(QMouseEvent * e);
	//void mouseMoveEvent(QMouseEvent *move);
	virtual void wheelEvent(QWheelEvent * event);

private:
};

#endif /* PAP_GUI_SRC_PADVIEW_H_ */
