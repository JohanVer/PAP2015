/********************************************************************************
 *
 ********************************************************************************/

#ifndef VERSIONSELECTORDIALOG_H
#define VERSIONSELECTORDIALOG_H

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
	void developerButton_clicked();
	void userButton_clicked();

private:
	Ui::VersionSelectorDialog *ui;

};

#endif // VERSIONSELECTORDIALOG_H
