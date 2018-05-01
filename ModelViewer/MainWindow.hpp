#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>

#include "ModelData.hpp"

class FileMenu;

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
  void file_open();
  void file_exit();
  void file_open_recent(const QString &filename);

private:
  Ui::MainWindow *ui;
  FileMenu* mFileMenu;
  ModelData *mModel;

  void read_settings();
  void save_settings();
};

#endif // MAINWINDOW_HPP
