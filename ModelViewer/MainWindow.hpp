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

private:
  Ui::MainWindow *ui;
  ModelData *mModel;

  void read_settings();
  void save_settings();
};

#endif // MAINWINDOW_HPP
