#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>

#include "ModelData.hpp"

class FileMenu;
class TestMenu;
class PrimitiveMenu;

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
  void file_open_recent(const QString &filename);
  void file_step();
  void file_stl();
  void file_exit();

  void test_all();
  void test_number();
  void test_script();
  void test_check();

  void primitive_block();
  void primitive_sphere();
  void primitive_cylinder();
  void primitive_cone();
  void primitive_torus();
  void primitive_NUBSurface();
  void primitive_line();
  void primitive_circle();
  void primitive_ellipse();
  void primitive_parabola();
  void primitive_hyperbola();
  void primitive_NUBcurve();

  //void on_actionCreateBlock_triggered();

  //void on_actionCreateSphere_triggered();

private:
  Ui::MainWindow *ui;
  FileMenu       *mFileMenu;
  TestMenu       *mTestMenu;
  PrimitiveMenu  *mPrimitiveMenu;
  ModelData      *mModel;

  void read_settings();
  void save_settings();
};

#endif // MAINWINDOW_HPP
