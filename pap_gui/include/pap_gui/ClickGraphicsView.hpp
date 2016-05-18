/*
 * ClickGraphicsView.h
 *
 *  Created on: Jul 4, 2015
 *      Author: johan
 */

#ifndef PAP_GUI_SRC_CLICKGRAPHICSVIEW_H_
#define PAP_GUI_SRC_CLICKGRAPHICSVIEW_H_

#include <QtWidgets>
#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QMouseEvent>

class ClickGraphicsView  : public QGraphicsView {
	Q_OBJECT
public:
	explicit ClickGraphicsView(QWidget *parent = 0);
	ClickGraphicsView();
	virtual ~ClickGraphicsView();

	Q_SIGNALS:


	public Q_SLOTS:
    virtual void wheelEvent(QWheelEvent * event);

	private:
	QGraphicsScene * scene;
};

class CameraGraphicsScene: public QGraphicsScene {
Q_OBJECT
public:
    explicit CameraGraphicsScene(QWidget *parent = 0);

Q_SIGNALS:
    void setFiducial(QPointF point);
    void sendMousePoint(QPointF point);

public Q_SLOTS:
void mousePressEvent(QGraphicsSceneMouseEvent *event);
void contextMenuEvent(QGraphicsSceneContextMenuEvent *event);

private:

};

#endif /* PAP_GUI_SRC_CLICKGRAPHICSVIEW_H_ */
