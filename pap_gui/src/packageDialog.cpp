#include <pap_gui/packageDialog.hpp>
#include <QDialogButtonBox>
#include <QMessageBox>
#include <stdio.h>
#include <stdlib.h>


PackageDialog::PackageDialog(QVector<databaseEntry> *database, QVector<std::string> *missingPackageList, int package, QWidget *parent) :
		QDialog(parent), ui(new Ui::PackageDialog) {

	ui->setupUi(this);
	databaseVector = database;
	packageNum = package;
	editPackage = false;

	// Add a new package - name given by missingPackageList
	if(missingPackageList != NULL && package != -1) {
		ui->nameLine->setText(QString::fromStdString(missingPackageList->at(package)));
	}

	// Edit an existing package
	if(missingPackageList == NULL) {
		editPackage = true;
		ui->nameLine->setText(database->at(package).package);
		ui->lengthLine->setText(QString::number(database->at(package).length));
		ui->heightLine->setText(QString::number(database->at(package).height));
		ui->widthLine->setText(QString::number(database->at(package).width));
		ui->pinsLine->setText(QString::number(database->at(package).pins));
	}
}

void PackageDialog::on_buttonBox_clicked(QAbstractButton* button) {
	// OK clicked to accept values
    if (ui->buttonBox->button(QDialogButtonBox::Ok) == (QPushButton*)button) {
    	bool ok;
    	QString package = ui->nameLine->text();
    	float length = ui->lengthLine->text().toFloat(&ok);
    	float height = ui->heightLine->text().toFloat();
    	float width = ui->widthLine->text().toFloat();
    	int pins = ui->pinsLine->text().toInt();

        // Check if all inputs are valid
        if(package.length() > 3 && length > 0 && height > 0 && width > 0 && pins > 0){

            // Edit given package
        	if(editPackage) {
        		(*databaseVector)[packageNum].package = package;
        		(*databaseVector)[packageNum].length = length;
        		(*databaseVector)[packageNum].height = height;
        		(*databaseVector)[packageNum].width = width;
        		(*databaseVector)[packageNum].pins = pins;
            // Add a new package
        	} else {
        		databaseEntry newEntry;
            	newEntry.package = package;
            	newEntry.length = length;
            	newEntry.height = height;
            	newEntry.width = width;
            	newEntry.pins = pins;
            	databaseVector->append(newEntry);
        	}
        } else {
    		QMessageBox msgBox;
    		msgBox.setText("Invalid input!");
    		msgBox.exec();
    		msgBox.close();
        }
    } else {
    	this->close();
    }
}

PackageDialog::~PackageDialog() {
	delete ui;
	delete databaseVector;
}
