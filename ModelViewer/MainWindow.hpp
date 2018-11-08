#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QTreeWidgetItem>

#include "ModelData.hpp"

class FileMenu;

class TestMenu;

class ViewMenu;

class PrimitiveMenu;

class TestDialog;

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

    ~MainWindow() override;

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:

    void file_open();

    void file_open_recent(const QString &filename);

    void file_sgm();

    void file_step();

    void file_stl();

    void file_exit();

    void view_faces();

    void view_edges();

    void view_vertices();

    void view_facet();

    void view_uvspace();

    void view_perspective();

    void view_background();

    void test_check();

#ifdef VIEWER_WITH_GTEST
    void test_gtest();
#endif

    void primitive_block();

    void primitive_sphere();

    void primitive_cylinder();

    void primitive_cone();

    void primitive_torus();

    void primitive_line();

    void primitive_circle();

    void primitive_ellipse();

    void primitive_parabola();

    void primitive_hyperbola();

    void primitive_NUBcurve();

    void primitive_torus_knot();

    void primitive_complex();

    void primitive_revolve();

private:

    Ui::MainWindow *ui;
    FileMenu *mFileMenu;
    ViewMenu *mViewMenu;
    TestMenu *mTestMenu;
    PrimitiveMenu *mPrimitiveMenu;
    ModelData *mModel;
    TestDialog *mTestDialog;

    void read_settings();

    void save_settings();
};

#endif // MAINWINDOW_HPP
