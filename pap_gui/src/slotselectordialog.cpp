#include "slotselectordialog.h"
#include "SlotGraphicsView.hpp"
#include "../../pap_placer/include/pap_placer/offsetTable.hpp"

SlotSelectorDialog::SlotSelectorDialog(QWidget *parent) :
		QDialog(parent), ui(new Ui::SlotSelectorDialog) {
	ui->setupUi(this);
	currentIndex_ = -1;
	QWidget::connect(&sceneSlots_, SIGNAL(sendMousePoint(int ,QPointF)), this,
			SLOT(slotPressed(int,QPointF)));

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
	paintSlots();
	for (size_t i = 0; i++; i < 88) {
		usedSlots_[i] = false;
	}
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

void SlotSelectorDialog::slotPressed(int numberOfFiducial, QPointF padPos) {
	int id = searchId(padPos);
	int currentPart = ui->partTable->currentRow();

	if (slotUsed(id)) {
		QMessageBox msgBox;
		msgBox.setText("This slot is already used.");
		msgBox.exec();
		msgBox.close();
	} else {
		partList[currentPart].slot = id;
		updatePartEntry(currentPart);
		updateComponentVector();
	}
	//currentIndex_ = id;
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

void SlotSelectorDialog::updatePartEntry(int updatedPartID) {
	ui->partTable->setItem(updatedPartID, 2,
			new QTableWidgetItem(
					QString::number(partList.at(updatedPartID).slot)));
}

int SlotSelectorDialog::getIndex() {
	return currentIndex_;
}

void SlotSelectorDialog::generatePartList(
		QVector<componentEntry> *componentVector) {

	// Set internal pointer to component vector
	componentVector_ = componentVector;
	bool inPartList;
	for (size_t i = 0; i < componentVector->size(); i++) {

		inPartList = false;
		for (size_t k = 0; k < partList.size(); k++) {
			// Check if component already in partList
			if (componentVector->at(i).package == partList.at(k).package
					&& componentVector->at(i).value == partList.at(k).value) {
				inPartList = true;
				partList.at(k).count++;
				(*componentVector)[i].box = partList.at(k).slot;
				break;
			}
		}

		if (!inPartList) {
			PartEntry partEntry;
			partEntry.package = componentVector->at(i).package;
			partEntry.value = componentVector->at(i).value;
			partEntry.count = 1;

			if (componentVector->at(i).box == -1) {
				partEntry.slot = -1;
			} else {
				if (slotUsed(componentVector->at(i).box)) {
					// Reset slot/box value
					partEntry.slot = -1;
					(*componentVector)[i].box = -1;

				} else {
					// Slot free, use it
					partEntry.slot = componentVector->at(i).box;
					usedSlots_[componentVector->at(i).box] = true;
				}
			}
			partList.push_back(partEntry);
		}
	}

	updatePartTable();
}

bool SlotSelectorDialog::slotUsed(int slotNumber) {

	bool returnVal = false;
	// Slot number not valid
	if (slotNumber > 86) {
		returnVal = true;
		// Slot already used
	} else if (usedSlots_[slotNumber]) {
		returnVal = true;
	}
	return returnVal;
}

void SlotSelectorDialog::updatePartTable() {

	// Set size of table
	ui->partTable->setRowCount(partList.size());
	ui->partTable->setColumnCount(4);

	// Set labels
	QStringList hLabels, vLabels;
	hLabels << "Package" << "Value" << "Slot" << "#";
	for (int i = 1; i < partList.size(); i++) {
		vLabels << QString::number(i);
	}

	ui->partTable->setHorizontalHeaderLabels(hLabels);
	ui->partTable->setVerticalHeaderLabels(vLabels);

	// Set content
	for (int i = 0; i < ui->partTable->rowCount(); i++) {
		ui->partTable->setItem(i, 0,
				new QTableWidgetItem((partList.at(i).package).c_str()));
		ui->partTable->setItem(i, 1,
				new QTableWidgetItem((partList.at(i).value).c_str()));
		if (partList.at(i).slot == -1) {
			ui->partTable->setItem(i, 2, new QTableWidgetItem("-"));
		} else {
			ui->partTable->setItem(i, 2,
					new QTableWidgetItem(QString::number(partList.at(i).slot)));
		}

		ui->partTable->setItem(i, 3,
				new QTableWidgetItem(QString::number(partList.at(i).count)));
	}

	// Table settings
	ui->partTable->sortItems(0, Qt::AscendingOrder);
	ui->partTable->setSelectionMode(QAbstractItemView::SingleSelection);
	ui->partTable->setSelectionBehavior(QAbstractItemView::SelectRows);
	ui->partTable->setSizePolicy(QSizePolicy::Expanding,
			QSizePolicy::Expanding);
	ui->partTable->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
	ui->partTable->show();
}

bool SlotSelectorDialog::getName(int index, std::string* name) {
	for (size_t i = 0; i < nameList.size(); i++) {
		if (nameList[i].index == index) {
			*name = nameList[i].name;
			return true;
		}
	}
	return false;
}

void SlotSelectorDialog::paintSlots(void) {
	static bool alreadyflipped = false;
	bool nameFound = false;
	std::string partName;
	sceneSlots_.clear();

	for (size_t i = 0; i < printedSlots_.size(); i++) {
		SlotInformation slot = printedSlots_[i];
		QGraphicsRectItem *rect = new QGraphicsRectItem(slot.pos.x(),
				slot.pos.y(), slot.pos.width(), slot.pos.height());
		if (getName(slot.index, &partName)) {
			nameFound = true;
			rect->setPen(QPen(Qt::green, 1, Qt::SolidLine));
			rect->setBrush(Qt::green);
		} else if (slot.occupied) {
			nameFound = false;
			rect->setPen(QPen(Qt::red, 1, Qt::SolidLine));
			rect->setBrush(Qt::red);
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
			text->setPlainText(partName.c_str());
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
	delete ui;
}
