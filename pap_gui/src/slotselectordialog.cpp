#include "slotselectordialog.h"
#include "SlotGraphicsView.hpp"
#include "../../pap_placer/include/pap_placer/offsetTable.hpp"

SlotSelectorDialog::SlotSelectorDialog(QWidget *parent) :
		QDialog(parent), ui(new Ui::SlotSelectorDialog) {
	ui->setupUi(this);
	currentIndex_ = -1;
	QWidget::connect(&sceneSlots_, SIGNAL(sendMousePoint(int ,QPointF)), this,
			SLOT(slotPressed(int,QPointF)));

	QObject::connect(ui->buttonBox, SIGNAL(clicked(QAbstractButton*)), this,
			SLOT(buttonBox_clicked(QAbstractButton*)));

	QWidget::connect(ui->mergeButton, SIGNAL(pressed()), this,
			SLOT(mergeButton_clicked()));
	QWidget::connect(ui->resetSlotButton, SIGNAL(pressed()), this,
			SLOT(resetSlotButton_clicked()));
	QWidget::connect(ui->showAllPartsButton, SIGNAL(pressed()), this,
			SLOT(showAllPartsButton_clicked()));
	QWidget::connect(ui->showMissingPartsButton, SIGNAL(pressed()), this,
			SLOT(showMissingPartsButton_clicked()));

	// Setup slot sizes/positions using offsetTable
	for (size_t i = 0; i < 47; i++) {
		SlotInformation slot;
		slot.pos.setX(BoxOffsetTable[i].y);
		slot.pos.setY(BoxOffsetTable[i].x);
		slot.pos.setWidth(10);
		slot.pos.setHeight(10);
		slot.index = i;
		printedSlots_.push_back(slot);
	}

	for (size_t i = 47; i < 59; i++) {
		SlotInformation slot;
		slot.pos.setX(BoxOffsetTable[i].y - 3);
		slot.pos.setY(BoxOffsetTable[i].x);
		slot.pos.setWidth(15);
		slot.pos.setHeight(15);
		slot.index = i;
		printedSlots_.push_back(slot);
	}

	for (size_t i = 59; i < 67; i++) {
		SlotInformation slot;
		slot.pos.setX(BoxOffsetTable[i].y - 4);
		slot.pos.setY(BoxOffsetTable[i].x);
		slot.pos.setWidth(20);
		slot.pos.setHeight(20);
		slot.index = i;
		printedSlots_.push_back(slot);
	}

	for (size_t i = 0; i < 20; i++) {
		SlotInformation slot;
		slot.pos.setX(TapeOffsetTable[i].y);
		slot.pos.setY(TapeOffsetTable[i].x);
		slot.pos.setWidth(5);
		slot.index = 67 + i;
		slot.pos.setHeight(30);
		printedSlots_.push_back(slot);
	}

	for (size_t i = 0; i < 88; i++) {
		usedSlots_[i] = false;
	}

	/* Full part list active at beginning */
	missingPartListActive = false;
	paintSlots();
	ui->graphicsView->scale(-1.5, 1.5);
}

int SlotSelectorDialog::searchId(QPointF position) {
	Q_EMIT setLed(-1);
	ros::Duration(0.5).sleep();
	QPointF convPoint;
	convPoint.setX(position.x());
	convPoint.setY(position.y());
	for (size_t i = 0; i < printedSlots_.size(); i++) {
		printedSlots_[i].occupied = false;
	}

	for (std::size_t i = 0; i < printedSlots_.size(); i++) {
		if (printedSlots_[i].pos.contains(convPoint)) {
			if (i < 59) {
				Q_EMIT setLed(i);
			}
			printedSlots_[i].occupied = true;
			paintSlots();
			return i;
		}
	}
	return -1;
}

void SlotSelectorDialog::buttonBox_clicked(QAbstractButton* button) {

	/*
	 if (ui->buttonBox->button(QDialogButtonBox::Apply) == button) {
	 ROS_INFO("ok button pressed");
	 } else if (ui->buttonBox->button(QDialogButtonBox::Abort) == button) {
	 ROS_INFO("cancel button pressed");
	 }
	 */
}

void SlotSelectorDialog::slotPressed(int numberOfFiducial, QPointF padPos) {
	int id = searchId(padPos);
	int partListIndex = ui->partTable->currentRow();
	int currentPartIndex = ui->partTable->currentRow();

	/* No part selected */
	if (currentPartIndex == -1) {
		QMessageBox msgBox;
		msgBox.setText("No part selected.");
		msgBox.exec();
		msgBox.close();
	}

	/* If missing parts shown -> calc. actual index */
	if (missingPartListActive) {
		/* Search for actual index in full list */
		for (size_t i = 0; i < partList.size(); i++) {
			if (partList[i].package == missingPartList[currentPartIndex].package
					&& partList[i].value
							== missingPartList[currentPartIndex].value) {
				partListIndex = i;
				break;
			}
		}
	}

	ROS_INFO("selected row: %d", ui->partTable->currentRow());
	ROS_INFO("actual row: %d", currentPartIndex);

	/* If not -> full list, index correct */
	if (slotUsed(id)) {
		QMessageBox msgBox;
		msgBox.setText("Slot in use - reset first!");
		msgBox.exec();
		msgBox.close();
	} else {
		/* If there has been set a slot before - reset usedSlots */
		if (partList[partListIndex].slot != -1) {
			usedSlots_[partList[partListIndex].slot] = false;
		}
		/* Update partList & used slots */
		partList[partListIndex].slot = id;
		usedSlots_[id] = true;
		/* Update table, image and initial comp. vector */
		if (missingPartListActive) {
			updateMissingPartList();
		}
		updateTable();
		paintSlots();
	}
}

void SlotSelectorDialog::updateComponentVector() {

	bool inPartList;
	ROS_INFO("internalCompVector size: %d", componentVector_->size());
	for (size_t i = 0; i < componentVector_->size(); i++) {

		inPartList = false;
		for (size_t k = 0; k < partList.size(); k++) {
			if (componentVector_->at(i).package == partList.at(k).package
					&& componentVector_->at(i).value == partList.at(k).value) {
				inPartList = true;
				(*componentVector_)[i].box = partList.at(k).slot;
				break;
			}
		}
		if (!inPartList) {
			(*componentVector_)[i].box = -1;
		}
	}
}

/* Updated slot of the given updatedPartID in table */
void SlotSelectorDialog::updatePartEntry(int updatedPartID) {
	if (partList.at(updatedPartID).slot == -1) {
		ui->partTable->setItem(updatedPartID, 2, new QTableWidgetItem("-"));
	} else {
		if (missingPartListActive) {
			ui->partTable->setItem(updatedPartID, 2,
					new QTableWidgetItem(
							QString::number(
									missingPartList.at(updatedPartID).slot)));
		} else {
			ui->partTable->setItem(updatedPartID, 2,
					new QTableWidgetItem(
							QString::number(partList.at(updatedPartID).slot)));
		}
	}
}

int SlotSelectorDialog::getIndex() {
	return currentIndex_;
}

bool sort_value(const PartEntry& element1, const PartEntry& element2) {
	return element1.value < element2.value;
}

bool sort_package(const PartEntry& element1, const PartEntry& element2) {
	return element1.package < element2.package;

}

void SlotSelectorDialog::generatePartList(
		QVector<componentEntry> *componentVector) {

// Set pointer to component vector
	componentVector_ = componentVector;
	bool inPartList;

	/* Iterate over all components*/
	for (size_t i = 0; i < componentVector->size(); i++) {

		inPartList = false;
		/* Store each type once, with its amount */
		for (size_t k = 0; k < partList.size(); k++) {

			/* Components equal if same package and value */
			if (componentVector->at(i).package == partList.at(k).package
					&& componentVector->at(i).value == partList.at(k).value) {

				/* If equal-increment count, add slot to component vector */
				inPartList = true;
				partList.at(k).count++;
				(*componentVector)[i].box = partList.at(k).slot;
				break;
			}
		}

		/* If component not yet in partlist */
		if (!inPartList) {

			/* New entry with package, value and count*/
			PartEntry partEntry;
			partEntry.package = componentVector->at(i).package;
			partEntry.value = componentVector->at(i).value;
			partEntry.count = 1;

			/* Check if component slot/box already set */
			int compSlotGiven = componentVector->at(i).box;
			if (compSlotGiven == -1) {
				partEntry.slot = -1;
			} else {
				/* If box number set, check if  */
				if (slotUsed(compSlotGiven)) {
					/* Reset part and component slot/box value */
					partEntry.slot = -1;
					(*componentVector)[i].box = -1;

				} else {
					/* Slot free, use it */
					partEntry.slot = compSlotGiven;
					usedSlots_[compSlotGiven] = true;
				}
			}
			partList.push_back(partEntry);
		}
	}

	/* Sort part list */
//std::sort(partList.begin(), partList.end(), sort_value);
	std::sort(partList.begin(), partList.end(), sort_package);

// TODO: Sort by values within each type of package

	/*Print usedSlots_ */
	for (int i = 0; i < 88; i++) {
		if (usedSlots_[i] == true) {
			ROS_INFO("usedSlot: %d: true \n", i);
		} else {
			ROS_INFO("usedSlot: %d: false \n", i);
		}
	}

	updateTable();
}

bool SlotSelectorDialog::slotUsed(int slotNumber) {

	/* Slot number not valid */
	if (slotNumber > 86) {
		return true;
	}
	/* Slot already used */
	else if (usedSlots_[slotNumber]) {
		return true;
	}
	return false;
}

void SlotSelectorDialog::updateTable() {

	std::vector<PartEntry>* list;
	if (missingPartListActive) {
		list = &missingPartList;
	} else {
		list = &partList;
	}

// Set size of table
	ui->partTable->setRowCount((*list).size());
	ui->partTable->setColumnCount(4);

// Set labels
	QStringList hLabels, vLabels;
	hLabels << "Package" << "Value" << "Slot" << "#";
	for (int i = 1; i < (*list).size(); i++) {
		vLabels << QString::number(i);
	}

	ui->partTable->setHorizontalHeaderLabels(hLabels);
	ui->partTable->setVerticalHeaderLabels(vLabels);

	/* Set content */
	for (int i = 0; i < ui->partTable->rowCount(); i++) {
		ui->partTable->setItem(i, 0,
				new QTableWidgetItem(((*list).at(i).package).c_str()));
		ui->partTable->setItem(i, 1,
				new QTableWidgetItem(((*list).at(i).value).c_str()));
		if ((*list).at(i).slot == -1) {
			ui->partTable->setItem(i, 2, new QTableWidgetItem("-"));
		} else {
			ui->partTable->setItem(i, 2,
					new QTableWidgetItem(QString::number((*list).at(i).slot)));
		}

		ui->partTable->setItem(i, 3,
				new QTableWidgetItem(QString::number((*list).at(i).count)));

		ROS_INFO("%s, %s, %d\n", ((*list).at(i).package).c_str(),
				((*list).at(i).value).c_str(), (*list).at(i).slot);
	}

// Table settings

	/*BE CAREFUL: This changes table order - but vector stays the same! Wrong indices!!*/
//ui->partTable->sortItems(0, Qt::AscendingOrder);
	ui->partTable->setSelectionMode(QAbstractItemView::SingleSelection);
	ui->partTable->setSelectionBehavior(QAbstractItemView::SelectRows);
	ui->partTable->setSizePolicy(QSizePolicy::Expanding,
			QSizePolicy::Expanding);
	ui->partTable->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
	ui->partTable->show();
}

void SlotSelectorDialog::resetSlotButton_clicked() {
	int currentPart = ui->partTable->currentRow();

	if (currentPart == -1) {
		QMessageBox msgBox;
		msgBox.setText("No part selected.");
		msgBox.exec();
		msgBox.close();
	} else {
		usedSlots_[partList[currentPart].slot] = false;
		partList[currentPart].slot = -1;
		updatePartEntry(currentPart);
		paintSlots();
	}
}

void SlotSelectorDialog::showAllPartsButton_clicked() {
	ROS_INFO("missingPartListActive = false");
	ui->resetSlotButton->setEnabled(true);
	missingPartListActive = false;
	updateTable();
}

void SlotSelectorDialog::showMissingPartsButton_clicked() {
	ROS_INFO("missingPartListActive = true");
	ui->resetSlotButton->setEnabled(false);
	missingPartListActive = true;
	updateMissingPartList();
	updateTable();
}

void SlotSelectorDialog::updateMissingPartList() {
	missingPartList.clear();
	for (int i = 0; i < partList.size(); i++) {
		if (partList.at(i).slot == -1) {
			missingPartList.push_back(partList.at(i));
		}
	}
}

void SlotSelectorDialog::mergeButton_clicked() {
	int currentPart = ui->partTable->currentRow();

	if (currentPart == -1) {
		QMessageBox msgBox;
		msgBox.setText("No part selected.");
		msgBox.exec();
		msgBox.close();
	} else {
		/*  */
		if (ui->mergeButton->text() == "Merge part") {
			ui->label_merge1->setText("currentPart");
			ui->mergeButton->setText("merge into");
			return;
		} else {
			/* Merge comp1 into comp2*/
			int comp1 = ui->label_merge1->text().toInt();
			int comp2 = currentPart; //ui->label_merge2->text();

			// Update partList
			partList[currentPart].count += partList[comp1].count;

			// Erase comp1
			std::vector<PartEntry> partList_temp;
			for (int i = 0; i < partList.size(); i++) {
				if (i != comp1) {
					partList_temp.push_back(partList.at(i));
				}
			}
			partList.clear();
			partList = partList_temp;

			ui->mergeButton->setText("Merge part");
		}
	}

}

void SlotSelectorDialog::getName(int SlotIndex, std::string* package,
		std::string* value) {
	for (size_t i = 0; i < partList.size(); i++) {
		if (partList[i].slot == SlotIndex) {
			*package = partList[i].package;
			*value = partList[i].value;
		}
	}
}

void SlotSelectorDialog::paintSlots(void) {
	static bool alreadyflipped = false;
	bool nameFound = false;
	std::string package = "Unknown";
	std::string value;
	sceneSlots_.clear();

	for (size_t i = 0; i < printedSlots_.size(); i++) {
		SlotInformation slot = printedSlots_[i];
		QGraphicsRectItem *rect = new QGraphicsRectItem(slot.pos.x(),
				slot.pos.y(), slot.pos.width(), slot.pos.height());

		/* NEW */
		if (slotUsed(slot.index)) {
			nameFound = true;
			getName(slot.index, &package, &value);
			rect->setPen(QPen(Qt::green, 1, Qt::SolidLine));
			rect->setBrush(Qt::green);
		} else {
			nameFound = false;
			rect->setPen(QPen(Qt::blue, 1, Qt::SolidLine));
			rect->setBrush(Qt::blue);
		}
		sceneSlots_.addItem(rect);

		// Print box numbers
		QGraphicsTextItem * text = new QGraphicsTextItem;
		text->setPos(slot.pos.x() + slot.pos.width(), slot.pos.y());
		text->scale(-1, 1);
		text->scale(0.2, 0.2);
		text->setDefaultTextColor(Qt::white);
		text->setPlainText(QString::number(i));
		sceneSlots_.addItem(text);

		// Print name
		if (nameFound) {
			QGraphicsTextItem * text = new QGraphicsTextItem;
			text->setPos(slot.pos.x() + slot.pos.width(), slot.pos.y() + 4);
			text->scale(-1, 1);
			text->scale(0.2, 0.2);
			text->setDefaultTextColor(Qt::white);
			if (slot.index >= 67) {
				text->rotate(90);
				text->setPos(text->pos().x() - 5, text->pos().y() + 4);
			}
			text->setPlainText(value.c_str());
			sceneSlots_.addItem(text);
		}
	}
	ui->graphicsView->setScene(&sceneSlots_);
	if (!alreadyflipped) {
		alreadyflipped = true;

	}
	ui->graphicsView->show();
}

SlotSelectorDialog::~SlotSelectorDialog() {
	updateComponentVector();
	delete ui;
}
