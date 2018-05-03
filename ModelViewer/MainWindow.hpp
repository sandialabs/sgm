#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>

#include "ModelData.hpp"

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

protected:
  void closeEvent(QCloseEvent *event);

private slots:
  void on_actionOpen_triggered();

  void on_actionExit_triggered();

  void on_actionRunAllTests_triggered();

  void on_actionRunTestNumber_triggered();

  void on_actionRunScript_triggered();

  void on_actionCreateBlock_triggered();

  void on_actionCreateSphere_triggered();

private:
  Ui::MainWindow *ui;
  ModelData *mModel;

  void read_settings();
  void save_settings();
};

#endif // MAINWINDOW_HPP
