#ifndef SLOTSELECTORDIALOG_H
#define SLOTSELECTORDIALOG_H

#include <QDialog>
#include"../../../pap_placer/include/pap_placer/placerClass.hpp"
#include <QGraphicsRectItem>
#include <QGraphicsObject>
#include <QMessageBox>
#include <QRectF>
#include "ui_slotselectordialog.h"
#include "CommonDataClasses.hpp"


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

class PartEntry{
public:
	std::string package, value;
	int count, slot;
	PartEntry(){
		count = 0;
		slot = 0;
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
	void generatePartList(QVector<componentEntry> *componentVector);
	void updatePartTable(void);
	bool getName(int index, std::string* name);
	bool slotUsed(int slotNumber);
	//void on_partTable_clicked();
	void updatePartEntry(int updatedPartID);
	void updateComponentVector();

	std::vector<SlotInformation> nameList;
	std::vector<PartEntry> partList;

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
	bool usedSlots_[87];
	int currentIndex_;
	QVector<componentEntry> *componentVector_;
};

#endif // SLOTSELECTORDIALOG_H

