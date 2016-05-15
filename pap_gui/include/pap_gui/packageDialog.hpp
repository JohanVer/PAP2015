/********************************************************************************
 *
 ********************************************************************************/

#ifndef PACKAGEDIALOG_H
#define PACKAGEDIALOG_H

#include <QtWidgets>
#include <QDialog>
#include <QVector>
#include "ui_packageDialog.h"
#include <pap_common/CommonDataClasses.hpp>

namespace Ui { class PackageDialog;}

class PackageDialog: public QDialog {
	Q_OBJECT

public:
	explicit PackageDialog(QVector<databaseEntry> *database, QVector<std::string> *missingPackageList, int package, QWidget *parent = 0);
	~PackageDialog();

public Q_SLOTS:
	void on_buttonBox_clicked(QAbstractButton* button);

private:
	Ui::PackageDialog *ui;
	QVector<databaseEntry> *databaseVector;
	int packageNum;
	bool editPackage;
};

#endif // PACKAGEDIALOG_H
