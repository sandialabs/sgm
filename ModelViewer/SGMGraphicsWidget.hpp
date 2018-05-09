#ifndef SGMGRAPHICSWIDGET_HPP
#define SGMGRAPHICSWIDGET_HPP

#include <memory>

#include "QVTKOpenGLWidget.h"
#include "SGMDataClasses.h"

class GView;
struct pGraphicsData;


class SGMGraphicsWidget : public QVTKOpenGLWidget
{
  Q_OBJECT
public:
  SGMGraphicsWidget(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
  ~SGMGraphicsWidget();

  void clear();

  void add_face(const std::vector<SGM::Point3D> &points,
                const std::vector<size_t> &triangles,
                const std::vector<SGM::UnitVector3D> &normals);

  void remove_faces();

  void add_edge(const std::vector<SGM::Point3D> &points);

  void reset_view();

private:
  pGraphicsData* dPtr;
};

#endif // SGMGRAPHICSWIDGET_HPP
