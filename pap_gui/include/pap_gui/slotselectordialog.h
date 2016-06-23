#ifndef SLOTSELECTORDIALOG_H
#define SLOTSELECTORDIALOG_H

#include <QtWidgets>
#include <QDialog>
#include <pap_placer/placerClass.hpp>
#include <QGraphicsRectItem>
#include <QGraphicsObject>
#include <QMessageBox>
#include <QRectF>
#include "ui_slotselectordialog.h"
#include <pap_common/CommonDataClasses.hpp>


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
    //!
    //! \brief SlotSelectorDialog implements an user dialog for selecting slots for all package types
    //! \param packageList list of all packages needed for currently loaded components
    //! \param databaseVector list of all component dimension information available
    //! \param parent QWidget
    //!
	explicit SlotSelectorDialog(QVector<componentEntry>* packageList, QVector<databaseEntry>* databaseVector, QWidget *parent = 0);
	~SlotSelectorDialog();

public Q_SLOTS:
    //!
    //! \brief slotPressed function, called if slot is pressed
    //! \param numberOfFiducial selected current fiducial (0 or 1)
    //! \param padPos x and y coordinates of selected pad
    //!
	void slotPressed(int numberOfFiducial, QPointF padPos);

    //!
    //! \brief on_resetSlotButton_clicked resets slot of currently selected package
    //!
	void on_resetSlotButton_clicked();

    //!
    //! \brief on_showAllPartsButton_clicked shows all parts, with and without assigend slot, in table
    //!
	void on_showAllPartsButton_clicked();

    //!
    //! \brief on_showMissingPartsButton_clicked shows only parts without assigned slot in table
    //!
	void on_showMissingPartsButton_clicked();

    //!
    //! \brief on_autoSlotSelectButton_clicked assignes slots to all missing parts automatically, based on part dimensions
    //!
	void on_autoSlotSelectButton_clicked();

    //!
    //! \brief on_buttonBox_clicked accept/cancel new slot assignements
    //!
	void on_buttonBox_clicked(QAbstractButton*);

Q_SIGNALS:

    //!
    //! \brief setLed turns selected LED on
    //! \param indexOfSlot index of LED to be turned on
    //!
	void setLed(int indexOfSlot);

private:

    //!
    //! \brief searchId searches for slot id clicked on
    //! \param position x and y coordinates of position clicked on
    //! \return slot Id if clicked on pad, otherwise -1
    //!
	int searchId(QPointF position);

    //!
    //! \brief getName searches package and value information of an assigned slot
    //! \param index assigned slot index
    //! \param package
    //! \param value
    //!
	void getName(int index, std::string* package, std::string* value);

    //!
    //! \brief paintSlots draws all slots into a scene
    //!
	void paintSlots(void);

    //!
    //! \brief updateTable updates table content
    //!
	void updateTable();

    //!
    //! \brief updatePartEntry
    //! \param updatedPartID id of part to be updated
    //!
	void updatePartEntry(int updatedPartID);

    //!
    //! \brief updateComponentVector takes all slots assigned within this dialog and
    //! assignes them finally to the corresponding components
    //!
	void updateComponentVector();

    //!
    //! \brief updateMissingPartList checks entire part list for parts without assigned slot
    //!
	void updateMissingPartList();

    //!
    //! \brief generatePartList generates a list of all parts/package types from entire component list
    //!
	void generatePartList();

    //!
    //! \brief getCompDimensions sreaches for dimensions of desired package and value
    //! \param package to be searched for
    //! \param value to be searched for
    //! \param length of package with desired value
    //! \param width of package with desired value
    //!
	void getCompDimensions(std::string package, std::string value,  float *length, float *width);

    //!
    //! \brief slotUsed check if given slot number is currently used
    //! \param slotNumber to be looked up
    //! \return true if slotNumber is used, otherwise false
    //!
	bool slotUsed(int slotNumber);

    //!
    //! \brief setSlot assignes given slot value to given component
    //! \param compIndex component id where slot is assigned to
    //! \param slot assigned slot id
    //!
	void setSlot(int compIndex, int slot);

	Ui::SlotSelectorDialog *ui;
	SlotGraphicsScene sceneSlots_;
	bool usedSlots_[87];
	std::vector<PartEntry> partList, missingPartList;
	bool missingPartListActive;
	std::vector<SlotInformation> printedSlots_;
	QVector<componentEntry> *componentVector_;
	QVector<databaseEntry>* databaseVector_;

    //!
    //! \brief safetyFactor ratio of box area alloes to be used and box area available
    //!
	float safetyFactor;

};

#endif // SLOTSELECTORDIALOG_H

