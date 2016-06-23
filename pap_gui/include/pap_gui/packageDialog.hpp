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
    //!
    //! \brief PackageDialog implements an user dialog for adding and editing database entries
    //! \param database current collection of component information
    //! \param missingPackageList list of all missing package
    //! \param package current package id in database
    //! \param parent QWidget
    //!
	explicit PackageDialog(QVector<databaseEntry> *database, QVector<std::string> *missingPackageList, int package, QWidget *parent = 0);

	~PackageDialog();

public Q_SLOTS:
    //!
    //! \brief on_accept_package_clicked updated package inforamtion or adds new package
    //! \param button
    //!
    void on_accept_package_clicked(QAbstractButton* button);

private:
	Ui::PackageDialog *ui;
	QVector<databaseEntry> *databaseVector;
	int packageNum;
	bool editPackage;
};

#endif // PACKAGEDIALOG_H
