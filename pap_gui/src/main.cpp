/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include <pap_gui/main_window.hpp>
#include <pap_gui/versionSelectorDialog.h>

/*****************************************************************************
** Qt application - Main
*****************************************************************************/

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    VersionSelectorDialog versionDialog;
    versionDialog.exec();
    pap_gui::MainWindow w(versionDialog.version, argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
    return result;
}
