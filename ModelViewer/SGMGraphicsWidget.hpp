#ifndef SGMGRAPHICSWIDGET_HPP
#define SGMGRAPHICSWIDGET_HPP

#include <QOpenGLWidget>
#include "SGMVector.h"

class GView;

struct GraphicsData;

namespace SGM {
    class Interval3D;
}

class SGMGraphicsWidget : public QOpenGLWidget
{
Q_OBJECT
public:
    explicit SGMGraphicsWidget(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());

    ~SGMGraphicsWidget() override;

    // Add triangles for a face to the internal buffer. The face will be rendered after
    // flush() has been called.
    void add_face(const std::vector<SGM::Point3D> &points,
                  const std::vector<unsigned int> &triangles,
                  const std::vector<SGM::UnitVector3D> &normals,
                  const std::vector<SGM::Vector3D> &colors);

    // Add points for an edge to the internal buffer. The edge will be rendered after
    // flush() has been called.
    void add_edge(std::vector<SGM::Point3D> const &points,
                  std::vector<SGM::Vector3D> const &colors);

    // Add triangle lines for a face to the internal buffer.
    // Face lines will be rendered after flush() has been called.
    void add_triangle_lines(const std::vector<SGM::Point3D> &points,
                            const std::vector<unsigned int> &triangles);

    // 2D version of triangle lines in UV space of a face
    void add_triangle_lines_uv(const std::vector<SGM::Point2D> &points,
                               const std::vector<unsigned int> &triangles);

    // Add the point for a vertex to the internal buffer. The vertex will be rendered after
    // flush() has been called.
    void add_vertex(SGM::Point3D const &Pos,
                    SGM::Vector3D const &ColorVec);

    // update bounds given axis-aligned bounding box
    void update_box_bounds(const SGM::Interval3D &box);

    // update bounds given point
    void update_point_bounds(const SGM::Point3D &point);

    // Flush the internal graphics buffer to push the data to the GPU and actually render.
    void flush();

    // Reset the camera perspective
    void reset_view();

    // Reset the bounding box.
    void reset_bounds();

    void set_render_vertices(bool render);

    void set_render_faces(bool render);

    void set_render_facets(bool render);

    void set_render_edges(bool render);

    void set_render_uvspace(bool render);

    void enable_perspective(bool enable);

    void set_background(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha);

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
    GraphicsData *dPtr;
};

#endif // SGMGRAPHICSWIDGET_HPP
