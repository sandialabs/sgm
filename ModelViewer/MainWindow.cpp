#include "MainWindow.hpp"
#include "ui_MainWindow.h"

#include <QApplication>
#include <QDesktopWidget>
#include <QFileDialog>
#include <QSettings>

#include "qinputdialog.h"
#include "qmessagebox.h"

#include "ModelData.hpp"

#include "SGMChecker.h"
#include "SGMPrimitives.h"
#include "SGMDataClasses.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  mModel(new ModelData)
{
  ui->setupUi(this);
  ui->mGraphics->setFocusPolicy(Qt::ClickFocus);

  mModel->set_tree_widget(ui->twTree);
  mModel->set_graphics_widget(ui->mGraphics);

  read_settings();
}

MainWindow::~MainWindow()
{
  delete ui;
  delete mModel;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  save_settings();
  QMainWindow::closeEvent(event);
}

void MainWindow::on_actionOpen_triggered()
{
  QString type_filter = tr("Step files (%1)").arg("*.stp *.step");
  type_filter += ";;" + tr("STL Files (%1)").arg("*.stl");
  type_filter += ";;" + tr("All files");

  // Run the dialog
  QStringList files = QFileDialog::getOpenFileNames(
        this, tr("Open File(s)"), "", type_filter);

  for(const QString &f : files)
    mModel->open_file(f);
}

void MainWindow::on_actionRunAllTests_triggered()
{
  QString DirectoryName=QFileDialog::getExistingDirectory(this, tr("Test Directory"), "");

  QString OutputName=QFileDialog::getSaveFileName(this, tr("Output File", ""));

  thing *pThing=SGM::CreateThing();
  SGM::Result rResult(pThing);
  SGM::RunTestDirectory(rResult,DirectoryName.toUtf8().data(),OutputName.toUtf8().data());
}

void MainWindow::on_actionRunTestNumber_triggered()
{
  int nTest=QInputDialog::getInt(this, tr("Test Number"), tr("C++ Test"));
  thing *pThing=SGM::CreateThing();
  SGM::Result rResult(pThing);
  if(SGM::RunCPPTest(rResult,nTest))
      {
      QMessageBox Msgbox;
        Msgbox.setText("Passed");
        Msgbox.exec();
      }
  else
      {
      QMessageBox Msgbox;
        Msgbox.setText("Failed");
        Msgbox.exec();
      }
}

void MainWindow::on_actionCreateBlock_triggered()
{
  //int nTest=QInputDialog::getInt(this, tr("Test Number"), tr("C++ Test"));

  SGM::Point3D Pos0(0,0,0);
  SGM::Point3D Pos1(2,4,9);
  mModel->create_block(Pos0,Pos1);
}

void MainWindow::on_actionCreateSphere_triggered()
{
  //int nTest=QInputDialog::getInt(this, tr("Test Number"), tr("C++ Test"));

  SGM::Point3D Pos0(0,0,0);
  mModel->create_sphere(Pos0,2.0);
}

void MainWindow::on_actionRunScript_triggered()
{
  QString FileName=QFileDialog::getOpenFileName(this, tr("Test Script"), "");

  thing *pThing=SGM::CreateThing();
  SGM::Result rResult(pThing);
  if(SGM::RunTestFile(rResult,"",FileName.toUtf8().data(),""))
      {
      QMessageBox Msgbox;
        Msgbox.setText("Passed");
        Msgbox.exec();
      }
  else
      {
      QMessageBox Msgbox;
        Msgbox.setText("Failed");
        Msgbox.exec();
      }
}

void MainWindow::on_actionExit_triggered()
{
  this->close();
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
