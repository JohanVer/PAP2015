/*
 * ClickGraphicsView.h
 *
 *  Created on: Jul 4, 2015
 *      Author: johan
 */

#ifndef PAP_GUI_SRC_PADVIEW_H_
#define PAP_GUI_SRC_PADVIEW_H_

#include <QtWidgets>
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
    void deletePad(QPointF point);
    void createPad(QRectF pad);


public Q_SLOTS:
void mousePressEvent(QGraphicsSceneMouseEvent *event);
void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
void contextMenuEvent(QGraphicsSceneContextMenuEvent *event);

private:
QGraphicsRectItem *temp_rect;
double x_press, y_press;
double width_press_, height_press_;
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
