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

//!
//! \brief The ClickGraphicsView class implements a
//! QGraphicsScene with wheelEvent handling
//!
class ClickGraphicsView  : public QGraphicsView {

    Q_OBJECT
    public:
	explicit ClickGraphicsView(QWidget *parent = 0);
	ClickGraphicsView();
	virtual ~ClickGraphicsView();

    Q_SIGNALS:
    public Q_SLOTS:

    //!
    //! \brief wheelEvent is a Q slot handling a QWheelEvent
    //! \param event occured
    //!
    virtual void wheelEvent(QWheelEvent * event);

    private:
	QGraphicsScene * scene;
};


//!
//! \brief The CameraGraphicsScene class implements
//! a GraphicsScene with event handling
//!
class CameraGraphicsScene: public QGraphicsScene {

    Q_OBJECT
    public:
    explicit CameraGraphicsScene(QWidget *parent = 0);

    Q_SIGNALS:
    //!
    //! \brief setFiducial is a Q signal passing a QPoint
    //! \param point passed by signal
    //!
    void setFiducial(QPointF point);

    //!
    //! \brief sendMousePoint is a Q signal passing a QPoint
    //! \param point passed by signal
    //!
    void sendMousePoint(QPointF point);

    public Q_SLOTS:
    //!
    //! \brief mousePressEvent is a Q slot handling a graphicSceneContextMenuEvent
    //! \param event occured
    //!
    void mousePressEvent(QGraphicsSceneMouseEvent *event);

    //!
    //! \brief contextMenuEvent is a Q slot handling a graphicSceneContextMenuEvent
    //! \param event occured
    //!
    void contextMenuEvent(QGraphicsSceneContextMenuEvent *event);
};

#endif /* PAP_GUI_SRC_CLICKGRAPHICSVIEW_H_ */
