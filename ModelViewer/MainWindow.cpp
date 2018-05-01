#include "MainWindow.hpp"
#include "ui_MainWindow.h"

#include <QApplication>
#include <QDesktopWidget>
#include <QFileDialog>
#include <QSettings>

#include "FileMenu.hpp"
#include "ModelData.hpp"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  mModel(new ModelData),
  mFileMenu(new FileMenu)
{
  ui->setupUi(this);
  ui->mGraphics->setFocusPolicy(Qt::ClickFocus);

  ui->menubar->addMenu(mFileMenu);
  connect(mFileMenu, SIGNAL(open()),
          this, SLOT(file_open()));
  connect(mFileMenu, SIGNAL(exit()),
          this, SLOT(file_exit()));
  connect(mFileMenu, SIGNAL(recent_file(QString)),
          this, SLOT(file_open_recent(QString)));

  mModel->set_tree_widget(ui->twTree);
  mModel->set_graphics_widget(ui->mGraphics);

  read_settings();
}

MainWindow::~MainWindow()
{
  delete mFileMenu;
  delete ui;

  delete mModel;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  save_settings();
  QMainWindow::closeEvent(event);
}

void MainWindow::file_open()
{
  QString type_filter = tr("Step files (%1)").arg("*.stp *.step");
  type_filter += ";;" + tr("STL Files (%1)").arg("*.stl");
  type_filter += ";;" + tr("All files");

  // Run the dialog
  QStringList files = QFileDialog::getOpenFileNames(
        this, tr("Open File(s)"), "", type_filter);

  for(const QString &f : files)
    file_open_recent(f);
}

void MainWindow::file_exit()
{
  this->close();
}

void MainWindow::file_open_recent(const QString &filename)
{
  bool opened = mModel->open_file(filename);
  if(opened)
    mFileMenu->add_recent_file(filename);
}

void MainWindow::read_settings()
{
  QSettings settings;
  if(settings.contains("MainWindow/geometry"))
    restoreGeometry(settings.value("MainWindow/geometry").toByteArray());
  else
  {
    // Center the main window
    QDesktopWidget* desktop = QApplication::desktop();
    QRect screen_geom = desktop->screenGeometry();
    QRect main_geom = geometry();
    move(screen_geom.center() - main_geom.center());
  }

  if(settings.contains("MainWindow/state"))
    restoreState(settings.value("MainWindow/state").toByteArray());
}

void MainWindow::save_settings()
{
  QSettings settings;
  settings.setValue("MainWindow/geometry", saveGeometry());
  settings.setValue("MainWindow/state", saveState());
}
