#ifndef SLOTSELECTORDIALOG_H
#define SLOTSELECTORDIALOG_H

#include <QDialog>
#include"../../../pap_placer/include/pap_placer/placerClass.hpp"
#include <QGraphicsRectItem>
#include <QGraphicsObject>
#include <QRectF>
#include "ui_slotselectordialog.h"

namespace Ui {
class SlotSelectorDialog;
}

class SlotInformation{
public:
	QRectF pos;
	int index;
	bool occupied;
	std::string name;
	SlotInformation(){
		occupied = false;
		index = 0;
	}
};

class SlotSelectorDialog: public QDialog {
Q_OBJECT

public:
	explicit SlotSelectorDialog(QWidget *parent = 0);
	~SlotSelectorDialog();
	int searchId(QPointF position);
	int getIndex();
	void paintSlots(void);
	bool getName(int index, std::string* name);

	std::vector<SlotInformation> nameList;
private:


Q_SIGNALS:
	void setSlotIndex(int indexOfSlot);
	void setLed(int indexOfSlot);

public Q_SLOTS:
	void slotPressed(int numberOfFiducial, QPointF padPos);

private:
	Ui::SlotSelectorDialog *ui;
	SlotGraphicsScene sceneSlots_;
	std::vector<SlotInformation> printedSlots_;
	int currentIndex_;
};

#endif // SLOTSELECTORDIALOG_H
