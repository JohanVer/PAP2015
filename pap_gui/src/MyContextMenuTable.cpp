/*
 * ContextMenuTable.cpp
 *
 *  Created on: Jul 5, 2015
 *      Author: johan
 */
// Johan Vertens & Nikolas Simon
// PAP2015 Student Projekt 6ECTS

#include <MyContextMenuTable.hpp>
#include "../include/pap_gui/main_window.hpp"


MyContextMenuTable::MyContextMenuTable(QWidget *parent) :
QTableWidget(parent),select(this->selectionModel())
{
	fiducialSize_ = 0;
}

MyContextMenuTable::~MyContextMenuTable() {
	// TODO Auto-generated destructor stub
}

void MyContextMenuTable::contextMenuEvent(QContextMenuEvent *event){
		QMenu menu(this);
	    QAction *gotoFiducial = new QAction("Goto fiducial", this);
	    QAction *deleteFiducial = new QAction("Delete fiducial", this);
	    menu.addAction(gotoFiducial);
	    menu.addAction(deleteFiducial);
	    QAction* selectedItem = menu.exec(event->globalPos());

	    if(selectedItem){
	    	if(selectedItem->text().toStdString() == "Goto fiducial" ){
	    		Q_EMIT sendGotoFiducial(select->selectedIndexes().at(0).row());

	    	}else if (selectedItem->text().toStdString() == "Delete fiducial"){
	    		int selectedRow = select->selectedIndexes().at(0).row();
	    		//if(selectedRow == 0 && fiducialSize_)
	    		this->setItem(selectedRow,0,new QTableWidgetItem(""));
	    		this->setItem(selectedRow,1,new QTableWidgetItem(""));
	    		this->setItem(selectedRow,2,new QTableWidgetItem(""));
	    		this->setItem(selectedRow,3,new QTableWidgetItem(""));
	    		fiducialSize_ = fiducialSize_ -1;
	    	}
	    }


}

