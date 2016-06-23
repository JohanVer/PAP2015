/********************************************************************************
 *
 ********************************************************************************/

#ifndef VERSIONSELECTORDIALOG_H
#define VERSIONSELECTORDIALOG_H

#include <QtWidgets>
#include <QDialog>
#include "ui_versionSelectorDialog.h"

namespace Ui { class VersionSelectorDialog;}

class VersionSelectorDialog: public QDialog {
	Q_OBJECT

public:
	explicit VersionSelectorDialog(QWidget *parent = 0);
	~VersionSelectorDialog();
	int version;

public Q_SLOTS:

    //!
    //! \brief developerButton_clicked set developer flag for GUI
    //!
	void developerButton_clicked();

    //!
    //! \brief userButton_clicked set user flag for GUI
    //!
	void userButton_clicked();

private:
	Ui::VersionSelectorDialog *ui;

};

#endif // VERSIONSELECTORDIALOG_H
