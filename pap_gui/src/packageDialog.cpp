#include "packageDialog.hpp"
#include <QDialogButtonBox>
#include <QMessageBox>


PackageDialog::PackageDialog(QWidget *parent) :
		QDialog(parent), ui(new Ui::PackageDialog) {

	ui->setupUi(this);
	allInputsValid = false;
	QObject::connect(ui->buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(on_buttonBox_clicked(QAbstractButton*)));
}

PackageDialog::PackageDialog(databaseEntry* entry, QWidget *parent) :
		QDialog(parent), ui(new Ui::PackageDialog) {

	ui->setupUi(this);
	allInputsValid = false;
	QObject::connect(ui->buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(on_buttonBox_clicked(QAbstractButton*)));
	newEntry = *entry;

}

void PackageDialog::on_buttonBox_clicked(QAbstractButton* button) {

	allInputsValid = false;

    if (ui->buttonBox->button(QDialogButtonBox::Ok) == (QPushButton*)button) {
    	newEntry.package = ui->nameLine->text();
    	newEntry.length = ui->lengthLine->text().toFloat();
    	newEntry.height = ui->heightLine->text().toFloat();
    	newEntry.width = ui->widthLine->text().toFloat();
    	newEntry.pins = ui->pinsLine->text().toInt();

        // Check if all inputs are valid
        if(newEntry.package.length() > 3 && newEntry.length > 0 && newEntry.height > 0 && newEntry.width > 0 && newEntry.pins > 0){
        	allInputsValid = true;
        } else {
    		QMessageBox msgBox;
    		msgBox.setText("Invalid input!");
    		msgBox.exec();
    		msgBox.close();
        }
    }
}

PackageDialog::~PackageDialog() {
	delete ui;
}
