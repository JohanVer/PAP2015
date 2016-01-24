#include "versionSelectorDialog.h"


VersionSelectorDialog::VersionSelectorDialog(QWidget *parent) :
		QDialog(parent), ui(new Ui::VersionSelectorDialog) {

	ui->setupUi(this);

	QWidget::connect(ui->developerButton, SIGNAL(pressed()), this,
				SLOT(developerButton_clicked()));
	QWidget::connect(ui->userButton, SIGNAL(pressed()), this,
				SLOT(userButton_clicked()));

	version = 0;		// Default version
}

void VersionSelectorDialog::developerButton_clicked() {
	version = 0;
	this->close();
}

void VersionSelectorDialog::userButton_clicked() {
	version = 1;
	this->close();
}

VersionSelectorDialog::~VersionSelectorDialog() {
	delete ui;
}
