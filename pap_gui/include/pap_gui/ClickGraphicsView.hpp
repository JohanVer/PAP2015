/*
 * ClickGraphicsView.h
 *
 *  Created on: Jul 4, 2015
 *      Author: johan
 */

#ifndef PAP_GUI_SRC_CLICKGRAPHICSVIEW_H_
#define PAP_GUI_SRC_CLICKGRAPHICSVIEW_H_

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
	void sendMousePoint(QPointF point);
	void setFiducial(QPointF point);

	public Q_SLOTS:
	void mousePressEvent(QMouseEvent * e);
	void mouseMoveEvent (QMouseEvent *move);
	void contextMenuEvent(QContextMenuEvent *event);

	private:
	QGraphicsScene * scene;
};

#endif /* PAP_GUI_SRC_CLICKGRAPHICSVIEW_H_ */
