#include "packageDialog.hpp"
#include <QDialogButtonBox>
#include <QMessageBox>

PackageDialog::PackageDialog(QVector<databaseEntry> *database, int package, QWidget *parent) :
		QDialog(parent), ui(new Ui::PackageDialog) {

	ui->setupUi(this);
	databaseVector = database;
	packageNum = package;

	// Init lineEdits with given values
	if(package != -1) {
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

        	// Add a new package
        	if(packageNum == -1) {
        		databaseEntry newEntry;
            	newEntry.package = package;
            	newEntry.length = length;
            	newEntry.height = height;
            	newEntry.width = width;
            	newEntry.pins = pins;
            	databaseVector->append(newEntry);
            // Edit given package
        	} else {
        		(*databaseVector)[packageNum].package = package;
        		(*databaseVector)[packageNum].length = length;
        		(*databaseVector)[packageNum].height = height;
        		(*databaseVector)[packageNum].width = width;
        		(*databaseVector)[packageNum].pins = pins;
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