#ifndef SLOTSELECTORDIALOG_H
#define SLOTSELECTORDIALOG_H

#include <QtWidgets>
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


class SlotSelectorDialog: public QDialog {
Q_OBJECT

public:
	explicit SlotSelectorDialog(QVector<componentEntry>* packageList, QVector<databaseEntry>* databaseVector, QWidget *parent = 0);
	~SlotSelectorDialog();

public Q_SLOTS:
	void slotPressed(int numberOfFiducial, QPointF padPos);
	void on_resetSlotButton_clicked();
	void on_showAllPartsButton_clicked();
	void on_showMissingPartsButton_clicked();
	void on_autoSlotSelectButton_clicked();
	void on_buttonBox_clicked(QAbstractButton*);

Q_SIGNALS:
	void setSlotIndex(int indexOfSlot);
	void setLed(int indexOfSlot);

private:
	int searchId(QPointF position);
	void getName(int index, std::string* package, std::string* value);
	void paintSlots(void);
	void updateTable();
	void updatePartEntry(int updatedPartID);
	void updateComponentVector();
	void updateMissingPartList();
	void generatePartList();
	void getCompDimensions(std::string package, std::string value,  float *length, float *width);
	bool slotUsed(int slotNumber);
	void setSlot(int compIndex, int slot);

	Ui::SlotSelectorDialog *ui;
	SlotGraphicsScene sceneSlots_;
	bool usedSlots_[87];
	std::vector<PartEntry> partList, missingPartList;
	bool missingPartListActive;
	std::vector<SlotInformation> printedSlots_;
	QVector<componentEntry> *componentVector_;
	QVector<databaseEntry>* databaseVector_;
	float safetyFactor;

};

#endif // SLOTSELECTORDIALOG_H

