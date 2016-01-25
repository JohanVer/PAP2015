/********************************************************************************
 *
 ********************************************************************************/

#ifndef PACKAGEDIALOG_H
#define PACKAGEDIALOG_H

#include <QDialog>
#include "ui_packageDialog.h"
#include "CommonDataClasses.hpp"

namespace Ui { class PackageDialog;}

class PackageDialog: public QDialog {
	Q_OBJECT

public:
	explicit PackageDialog(QWidget *parent = 0);
	PackageDialog(databaseEntry* entry, QWidget *parent);
	~PackageDialog();
	databaseEntry newEntry;

public Q_SLOTS:
	void on_buttonBox_clicked(QAbstractButton* button);

private:
	Ui::PackageDialog *ui;
	bool allInputsValid;

};

#endif // PACKAGEDIALOG_H
