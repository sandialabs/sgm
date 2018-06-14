
#include <memory>

#include <QApplication>
#include <QSettings>
#include <QSurfaceFormat>

#include "MainWindow.hpp"
#include "QVTKOpenGLWidget.h"
#include "vtkGenericOpenGLRenderWindow.h"

int main(int argc, char** argv)
{
  // Setup some defaults for the settings file
  QApplication::setOrganizationName("Sandia");
  QApplication::setApplicationName("SGM-Model-Viewer");
  QSettings::setDefaultFormat(QSettings::IniFormat);

  // The surface format should be set before we instantiate the
  // QApplication
  QSurfaceFormat format = QVTKOpenGLWidget::defaultFormat();
  format.setSwapInterval(0);
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());

  QApplication app(argc, argv);

  // Setup the main window
  MainWindow mw;
  mw.show();

  // Run the application
  return QApplication::exec();
}

