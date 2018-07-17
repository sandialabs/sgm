#include <QApplication>
#include <QSettings>
#include <QSurfaceFormat>

#include "MainWindow.hpp"
#include "SGMGraphicsWidget.hpp"

int main(int argc, char **argv)
{
    // Setup some defaults for the settings file
    QApplication::setOrganizationName("Sandia");
    QApplication::setApplicationName("SGM-Model-Viewer");
    QSettings::setDefaultFormat(QSettings::IniFormat);

    // The surface format should be set before we instantiate the
    // QApplication
    QSurfaceFormat::setDefaultFormat(SGMGraphicsWidget::default_format());

    QApplication app(argc, argv);

    // Setup the main window
    MainWindow mw;
    mw.show();

    // Run the application
    return QApplication::exec();
}

