#ifndef SGMGRAPHICSWIDGET_HPP
#define SGMGRAPHICSWIDGET_HPP

#include <memory>

#include "QVTKOpenGLWidget.h"

class GView;

class SGMGraphicsWidget : public QVTKOpenGLWidget
{
  Q_OBJECT
public:
  SGMGraphicsWidget(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
  ~SGMGraphicsWidget();

private:
  std::shared_ptr<GView> mView;
};

#endif // SGMGRAPHICSWIDGET_HPP
