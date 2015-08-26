#include "slotselectordialog.h"
#include "SlotGraphicsView.hpp"

const Offset SmallBoxOffsetTable[59] =
		{ { 0.0, 0.0 }, { 16.66, 0.0 }, { 33.32, 0.0 }, { 49.98, 0.0 }, { 66.64,
				0.0 }, { 83.30, 0.0 }, { 99.96, 0.0 }, { 116.62, 0.0 }, {
				133.28, 0.0 }, { 149.94, 0.0 }, { 166.60, 0.0 },
				{ 166.60, -14.0 }, { 149.94, -14.0 }, { 133.28, -14.0 }, {
						116.62, -14.0 }, { 99.96, -14.0 }, { 83.30, -14.0 }, {
						66.64, -14.0 }, { 49.98, -14.0 }, { 33.32, -14.0 }, {
						16.66, -14.0 }, { 0, -14.0 }, { 0.0, -28.0 }, { 16.66,
						-28.0 }, { 33.32, -28.0 }, { 49.98, -28.0 }, { 66.64,
						-28.0 }, { 66.64, -42.0 }, { 49.98, -42.0 }, { 33.32,
						-42.0 }, { 16.66, -42.0 }, { 0.0, -42.0 }, { 0, -56.0 },
				{ 16.66, -56.0 }, { 33.32, -56.0 }, { 49.98, -56.0 }, { 66.64,
						-56.0 }, { 66.64, -70.0 }, { 49.98, -70.0 }, { 33.32,
						-70.0 }, { 16.66, -70.0 }, { 0.0, -70.0 }, { 0, -84.0 },
				{ 16.66, -84.0 }, { 33.32, -84.0 }, { 49.98, -84.0 }, { 66.64,
						-84.0 }, { 187.65, -2.0 }, { 187.65, -18.6 }, { 187.65,
						-35.2 }, { 187.65, -51.8 }, { 187.65, -68.4 }, { 187.65,
						-85 }, { 204.25, -2.0 }, { 204.25, -18.6 }, { 204.25,
						-35.2 }, { 204.25, -51.8 }, { 204.25, -68.4 }, { 204.25,
						-85 } };

int SlotSelectorDialog::searchId(QPointF position) {
	Q_EMIT setLed(-1);
	ros::Duration(0.2).sleep();
	QPointF convPoint;
	convPoint.setX(position.x());
	convPoint.setY(position.y());
	for (size_t i = 0; i < printedSlots_.size(); i++) {
		printedSlots_[i].occupied = false;
	}
	// First rect is outline
	for (std::size_t i = 1; i < printedSlots_.size(); i++) {
		if (printedSlots_[i].pos.contains(convPoint)) {
			Q_EMIT setLed(i);
			printedSlots_[i].occupied = true;
			paintSlots();
			return i;
		}
	}
	return -1;
}

void SlotSelectorDialog::slotPressed(int numberOfFiducial, QPointF padPos) {
	int id = searchId(padPos);
	currentIndex_ = id;
}

int SlotSelectorDialog::getIndex() {
	return currentIndex_;
}

SlotSelectorDialog::SlotSelectorDialog(QWidget *parent) :
		QDialog(parent), ui(new Ui::SlotSelectorDialog) {
	ui->setupUi(this);
	currentIndex_ = -1;
	QWidget::connect(&sceneSlots_, SIGNAL(sendMousePoint(int ,QPointF)), this,
			SLOT(slotPressed(int,QPointF)));
	for (size_t i = 0; i < 59; i++) {
		SlotInformation slot;
		slot.pos.setX(SmallBoxOffsetTable[i].y);
		slot.pos.setY(SmallBoxOffsetTable[i].x);
		slot.pos.setWidth(10);
		slot.pos.setHeight(10);
		printedSlots_.push_back(slot);
	}
	paintSlots();

}

void SlotSelectorDialog::paintSlots(void) {
	static bool alreadyflipped = false;
	sceneSlots_.clear();
	for (size_t i = 0; i < printedSlots_.size(); i++) {
		SlotInformation slot = printedSlots_[i];
		QGraphicsRectItem *rect = new QGraphicsRectItem(
				SmallBoxOffsetTable[i].y, SmallBoxOffsetTable[i].x, 10, 10);
		if (slot.occupied) {
			rect->setPen(QPen(Qt::green, 1, Qt::SolidLine));
			rect->setBrush(Qt::green);
		} else {
			rect->setPen(QPen(Qt::blue, 1, Qt::SolidLine));
			rect->setBrush(Qt::blue);
		}
		sceneSlots_.addItem(rect);
	}
	ui->graphicsView->setScene(&sceneSlots_);
	if (!alreadyflipped) {
		alreadyflipped = true;
		ui->graphicsView->scale(-1, 1);
	}
	ui->graphicsView->show();
}

SlotSelectorDialog::~SlotSelectorDialog() {
	delete ui;
}
