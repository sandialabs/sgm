#include "SGMGraphicsWidget.hpp"



#include "vtkGenericOpenGLRenderWindow.h"
#include "GView.hpp"

SGMGraphicsWidget::SGMGraphicsWidget(QWidget *parent, Qt::WindowFlags f) :
  QVTKOpenGLWidget(parent, f),
  mView(new GView)
{
  vtkGenericOpenGLRenderWindow* myrenwin = vtkGenericOpenGLRenderWindow::New();
  this->SetRenderWindow(myrenwin);
  myrenwin->Delete();

  mView->set_render_window(this->GetRenderWindow());
  mView->reset_camera();
}

SGMGraphicsWidget::~SGMGraphicsWidget()
{

}
