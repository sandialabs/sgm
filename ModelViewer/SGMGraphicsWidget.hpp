#ifndef SGMGRAPHICSWIDGET_HPP
#define SGMGRAPHICSWIDGET_HPP

#include <QOpenGLWidget>
#include "SGMVector.h"

class GView;
struct pGraphicsData;

namespace SGM
{
    class Interval3D;
}

class SGMGraphicsWidget : public QOpenGLWidget
{
  Q_OBJECT
public:
  SGMGraphicsWidget(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
  ~SGMGraphicsWidget();

  // Add edges to internal buffer, the edges will be rendered after flush() has been called.
  void add_edge(const std::vector<SGM::Point3D> &points);

  // Add triangles for a face to the internal buffer. The face will be rendered after
  // flush() has been called.
  void add_face(const std::vector<SGM::Point3D> &points,
                const std::vector<unsigned int> &triangles,
                const std::vector<SGM::UnitVector3D> &normals);

  // Add triangle lines for a face to the internal buffer.
  // Face lines will be rendered after flush() has been called.
  void add_triangle_lines(const std::vector<SGM::Point3D> &points,
                          const std::vector<unsigned int> &triangles);

  // 2D version of triangle lines in UV space of a face
  void add_triangle_lines_uv(const std::vector<SGM::Point2D> &points,
                             const std::vector<unsigned int> &triangles);

  // update bounds given axis-aligned bounding box
  void update_bounds(const SGM::Interval3D& box);

  // Flush the internal graphics buffer to push the data to the GPU and actually render.
  void flush();

  // Reset the camera perspective
  void reset_view();

  void set_render_faces(bool render);
  void enable_perspective(bool enable);

  // Get the default format that the QApplication needs for rendering
  static QSurfaceFormat default_format();

protected:
  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int w, int h) override;

  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;

  void exec_context_menu(const QPoint &pos);

private:

  pGraphicsData* dPtr;
};

#endif // SGMGRAPHICSWIDGET_HPP
