/*
 * ContextMenuTable.hpp
 *
 *  Created on: Jul 5, 2015
 *      Author: johan
 */

#ifndef PAP_GUI_SRC_MYCONTEXTMENUTABLE_HPP_
#define PAP_GUI_SRC_MYCONTEXTMENUTABLE_HPP_

//#include "main_window.hpp"
#include <QWidget>
#include <QPointF>
#include <QDebug>
#include <QMouseEvent>
#include <QTableWidget>
#include <QItemSelectionModel>

class MyContextMenuTable: public QTableWidget {
	Q_OBJECT
public:
	explicit MyContextMenuTable(QWidget *parent = 0);
	virtual ~MyContextMenuTable();

	Q_SIGNALS:
	void sendGotoFiducial(int indexOfFiducial);

	public Q_SLOTS:
	void contextMenuEvent(QContextMenuEvent *event);

	public:
	int fiducialSize_;

	private:
	QItemSelectionModel *select;


};

#endif /* PAP_GUI_SRC_MYCONTEXTMENUTABLE_HPP_ */