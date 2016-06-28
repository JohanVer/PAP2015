#include <pap_gui/slotselectordialog.h>
#include <pap_gui/SlotGraphicsView.hpp>
#include "../../pap_placer/include/pap_placer/offsetTable.hpp"

SlotSelectorDialog::SlotSelectorDialog(QVector<componentEntry>* packageList, QVector<databaseEntry>* database, QWidget *parent) :
		QDialog(parent), ui(new Ui::SlotSelectorDialog) {
	ui->setupUi(this);
	QWidget::connect(&sceneSlots_, SIGNAL(sendMousePoint(int ,QPointF)), this,
			SLOT(slotPressed(int,QPointF)));

	for (size_t i = 0; i < 47; i++) {
		SlotInformation slot;
        slot.pos.setX(-BoxOffsetTable[i].y);
		slot.pos.setY(BoxOffsetTable[i].x);
		slot.pos.setWidth(10);
		slot.pos.setHeight(10);
		slot.index = i;
		printedSlots_.push_back(slot);
	}

	for (size_t i = 47; i < 59; i++) {
		SlotInformation slot;
        slot.pos.setX(-BoxOffsetTable[i].y - 3);
		slot.pos.setY(BoxOffsetTable[i].x);
		slot.pos.setWidth(15);
		slot.pos.setHeight(15);
		slot.index = i;
		printedSlots_.push_back(slot);
	}

	for (size_t i = 59; i < 67; i++) {
		SlotInformation slot;
        slot.pos.setX(-BoxOffsetTable[i].y - 4);
		slot.pos.setY(BoxOffsetTable[i].x);
		slot.pos.setWidth(20);
		slot.pos.setHeight(20);
		slot.index = i;
		printedSlots_.push_back(slot);
	}

	for (size_t i = 0; i < 20; i++) {
		SlotInformation slot;
        slot.pos.setX(-TapeOffsetTable[i].y);
		slot.pos.setY(TapeOffsetTable[i].x);
		slot.pos.setWidth(5);
		slot.index = 67 + i;
		slot.pos.setHeight(30);
		printedSlots_.push_back(slot);
	}

	for (size_t i = 0; i < 88; i++) { usedSlots_[i] = false;}

    // Both slots used for QR-Calibration
    usedSlots_[0] = true;
    usedSlots_[58] = true;
    // Both slots used for checkerboard calibration
    usedSlots_[65] = true;
    usedSlots_[66] = true;

	componentVector_ = packageList;
	databaseVector_ = database;
	missingPartListActive = false;
	safetyFactor = 0.8;
	generatePartList();
	paintSlots();
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

void SlotSelectorDialog::on_buttonBox_clicked(QAbstractButton* button) {

	if (ui->buttonBox->button(QDialogButtonBox::Ok) == (QPushButton*)button) {
		// Save slots!
		updateComponentVector();
	} else {
		// Discard slot selection (don't update componentVector)
		this->close();
	}
}

void SlotSelectorDialog::slotPressed(int numberOfFiducial, QPointF padPos) {
	int id = searchId(padPos);
	if(id < 0) return;
	int partListIndex = ui->partTable->currentRow();
	int currentPartIndex = ui->partTable->currentRow();

	/* No part selected */
	if (currentPartIndex == -1) {
		QMessageBox msgBox;
		msgBox.setText("No part selected.");
		msgBox.exec();
		msgBox.close();
    } else {

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
}

void SlotSelectorDialog::updateComponentVector() {

	bool inPartList;
	// Iterate over entire component vector
	for (size_t i = 0; i < componentVector_->size(); i++) {
		inPartList = false;
		//Find its slot in partList
		for (size_t k = 0; k < partList.size(); k++) {
			//Update slot of current component if in partList
			if (componentVector_->at(i).package == partList.at(k).package
					&& componentVector_->at(i).value == partList.at(k).value) {
				inPartList = true;
				(*componentVector_)[i].box = partList.at(k).slot;
				break;
			}
		}
		// Component not in partList - now slot known
		if (!inPartList) {
				(*componentVector_)[i].box = -1;
		}
	}
}

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

bool sort_value(const PartEntry& element1, const PartEntry& element2) {
	return element1.value < element2.value;
}

bool sort_package(const PartEntry& element1, const PartEntry& element2) {
	return element1.package < element2.package;
}

void SlotSelectorDialog::generatePartList() {

	bool inPartList;
	/* Iterate over all components*/
	for (size_t i = 0; i < componentVector_->size(); i++) {
		inPartList = false;
		/* Store each type once, with its amount */
		for (size_t k = 0; k < partList.size(); k++) {

			/* Components equal if same package and value */
			if (componentVector_->at(i).package == partList.at(k).package
					&& componentVector_->at(i).value == partList.at(k).value) {

				/* Increment count, add slot to component vector ??????can this happen???*/
				inPartList = true;
				partList.at(k).count++;
				(*componentVector_)[i].box = partList.at(k).slot;
				break;
			}
		}

		/* If component not yet in partlist */
		if (!inPartList) {

			/* New entry with package, value and count*/
			PartEntry partEntry;
			partEntry.package = componentVector_->at(i).package;
			partEntry.value = componentVector_->at(i).value;
			partEntry.count = 1;

			/* Check if component slot/box already set */
			int compSlotGiven = componentVector_->at(i).box;

			if (compSlotGiven == -1) {
				partEntry.slot = -1;
			} else {
				/* If box number set, check if  */
				if (slotUsed(compSlotGiven)) {
					/* Reset part and component slot/box value */
					partEntry.slot = -1;
					(*componentVector_)[i].box = -1;

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
	updateTable();
}

bool SlotSelectorDialog::slotUsed(int slotNumber) {

	/* Slot number not valid or used */
	if (slotNumber > 86) {
		return true;
	} else if (usedSlots_[slotNumber]) {
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

	// Set size of table and labels
	ui->partTable->setRowCount((*list).size());
	ui->partTable->setColumnCount(4);
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
	}

	/*BE CAREFUL: This changes table order - but vector stays the same! Wrong indices!!*/
	//ui->partTable->sortItems(0, Qt::AscendingOrder);
	ui->partTable->setSelectionMode(QAbstractItemView::SingleSelection);
	ui->partTable->setSelectionBehavior(QAbstractItemView::SelectRows);
	ui->partTable->setSizePolicy(QSizePolicy::Expanding,
			QSizePolicy::Expanding);
    ui->partTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	ui->partTable->show();
}

void SlotSelectorDialog::on_resetSlotButton_clicked() {
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

void SlotSelectorDialog::on_showAllPartsButton_clicked() {
	ui->resetSlotButton->setEnabled(true);
	missingPartListActive = false;
	updateTable();
}

void SlotSelectorDialog::on_showMissingPartsButton_clicked() {
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
	bool nameFound = false;
	std::string package = "Unknown";
	std::string value;
	sceneSlots_.clear();

	for (size_t i = 0; i < printedSlots_.size(); i++) {
		SlotInformation slot = printedSlots_[i];
        QGraphicsRectItem *rect = new QGraphicsRectItem(slot.pos.x(),
                slot.pos.y(), slot.pos.width(), slot.pos.height());

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
        text->setPos((slot.pos.x()),  slot.pos.y());
        text->setScale(0.2);
        text->scale();
        text->setDefaultTextColor(Qt::black);
        text->setPlainText(QString::number(i));
        sceneSlots_.addItem(text);

        // Print name
        if (nameFound) {
            QGraphicsTextItem * text = new QGraphicsTextItem;
            text->setPos((slot.pos.x()), (slot.pos.y() + 4));
            text->setScale(0.2);
            text->setDefaultTextColor(Qt::black);
            if (slot.index >= 67) {
                text->setPos((text->pos().x() - 5), (text->pos().y() + 4));
            }
            text->setPlainText(value.c_str());
            sceneSlots_.addItem(text);
        }
    }

	ui->graphicsView->setScene(&sceneSlots_);
	ui->graphicsView->show();
}

void SlotSelectorDialog::on_autoSlotSelectButton_clicked() {
	// Update missing part List
	missingPartListActive = true;
	updateMissingPartList();

	bool allFound = true;
	// Iterate over missingPartList
	for (size_t part_index=0; part_index < missingPartList.size(); part_index++) {

		/* Get package information */
		float length, width;
		getCompDimensions(missingPartList.at(part_index).package, missingPartList.at(part_index).value, &length, &width);
		int compNumber = missingPartList.at(part_index).count;
		float totCompArea = length * width * compNumber;
		/* First try: Small boxes */
        ROS_ERROR("CompArea: %f", totCompArea);
        if (totCompArea < safetyFactor * 100.00) {
			for (size_t i = 0; i < 47; i++) {
				if(usedSlots_[i] == false){
					setSlot(part_index, i);
					break;
				}
			}

		/* Second try: Middle boxes */
        } else if (totCompArea < safetyFactor * 225.00) {
			for (size_t i = 47; i < 59; i++) {
				if(usedSlots_[i] == false){
					setSlot(part_index, i);
					break;
				}
			}

		/* Third try: Large boxes */
        } else if (totCompArea < safetyFactor * 400.00){
			for (size_t i = 59; i < 67; i++) {
				if(usedSlots_[i] == false){
					setSlot(part_index, i);
					break;
				}
			}
		} else {

			allFound = false;
		}
	}

	if(!allFound){
		updateMissingPartList();
		QMessageBox msgBox;
        msgBox.setText("There are parts left that don't fit into any slot!");
		msgBox.exec();
		msgBox.close();
	} else {
		missingPartListActive = false;
	}
	updateTable();
	paintSlots();
}

int SlotSelectorDialog::rightTipSlotAvailable(float requiredArea) {
    /* First try: Small boxes */
    if (requiredArea < safetyFactor * 100.00) {
        for (size_t i = 37; i < 47; i++) {
            if(usedSlots_[i] == false)
                return i;
        }
    /* Second try: Middle boxes */
    } else if (requiredArea < safetyFactor * 225.00) {
        if(usedSlots_[51] == false) {
            return 51;
        } else if(usedSlots_[52] == false) {
            return 52;
        } else if(usedSlots_[57] == false) {
            return 57;
        } else if(usedSlots_[58] == false) {
            return 58;
        }
    /* Third try: Large boxes */
    } else if (requiredArea < safetyFactor * 400.00){
        if(usedSlots_[61] == false) {
            return 61;
        } else if(usedSlots_[62] == false) {
            return 62;
        } else if(usedSlots_[65] == false) {
            return 65;
        } else if(usedSlots_[66] == false) {
            return 66;
        }
    }
    return -1;
}

int SlotSelectorDialog::leftTipSlotAvailable(float requiredArea) {
    /* First try: Small boxes */
    if (requiredArea < safetyFactor * 100.00) {
        for (size_t i = 0; i < 37; i++) {
            if(usedSlots_[i] == false)
                return i;
        }
    /* Second try: Middle boxes */
    } else if (requiredArea < safetyFactor * 225.00) {
        for (size_t i = 47; i < 51; i++) {
            if(usedSlots_[i] == false)
                return i;
        }
        for (size_t i = 53; i < 57; i++) {
            if(usedSlots_[i] == false)
                return i;
        }
    /* Third try: Large boxes */
    } else if (requiredArea < safetyFactor * 400.00){
        if(usedSlots_[59] == false) {
            return 59;
        } else if(usedSlots_[60] == false) {
            return 60;
        } else if(usedSlots_[63] == false) {
            return 63;
        } else if(usedSlots_[64] == false) {
            return 64;
        }
    }
}

void SlotSelectorDialog::on_equalizedAutoSlotSelectButton_clicked() {
    // Update missing part List
    missingPartListActive = true;
    updateMissingPartList();

    int leftTipComponentCount = 0;
    int rightTipComponentCount = 0;

    bool allFound = true;
    // Iterate over missingPartList
    for (size_t part_index=0; part_index < missingPartList.size(); part_index++) {

        /* Get package information */
        float length, width;
        getCompDimensions(missingPartList.at(part_index).package, missingPartList.at(part_index).value, &length, &width);
        int compNumber = missingPartList.at(part_index).count;
        float totCompArea = length * width * compNumber;

        int slotID = rightTipSlotAvailable(totCompArea);
        if((rightTipComponentCount <= leftTipComponentCount) && slotID != -1) {
            setSlot(part_index, slotID);
            rightTipComponentCount += compNumber;
        } else {
            slotID = leftTipSlotAvailable(totCompArea);
            if(slotID > -1) {
                setSlot(part_index, slotID);
                leftTipComponentCount += compNumber;
            } else {
                allFound = false;
            }
        }
    }

    if(!allFound){
        updateMissingPartList();
        QMessageBox msgBox;
        msgBox.setText("There are parts left that don't fit into any slot!");
        msgBox.exec();
        msgBox.close();
    } else {
        missingPartListActive = false;
    }
    updateTable();
    paintSlots();
}

void SlotSelectorDialog::setSlot(int compIndex, int slot) {

	int index = compIndex;
	if (missingPartListActive) {
		/* Search for actual index in full list */
		for (size_t i = 0; i < partList.size(); i++) {
			if (partList[i].package == missingPartList[index].package
					&& partList[i].value
							== missingPartList[index].value) {
				index = i;
				break;
			}
		}
	}

	/* Update partList & used slots */
	partList[index].slot = slot;
	usedSlots_[slot] = true;
}

void SlotSelectorDialog::getCompDimensions(std::string package, std::string value,  float *length, float *width) {
	QString qPackage = QString::fromStdString(package);
	QString qValue = QString::fromStdString(value);

	for (size_t i = 0; i < databaseVector_->size(); i++) {
		// Compare value missing!!!!
		if (databaseVector_->at(i).package.compare(qPackage) == 0) {
			(*length) = databaseVector_->at(i).length;
			(*width) = databaseVector_->at(i).width;
			break;
		}
	}
}

SlotSelectorDialog::~SlotSelectorDialog() {
	missingPartList.clear();
	partList.clear();
	delete ui;
}
