#ifndef PRIMITIVIEMENU_HPP
#define PRIMITIVIEMENU_HPP

#include <QMenu>

class PrimitiveMenu : public QMenu
{
  Q_OBJECT
public:
  PrimitiveMenu(QWidget *parent=Q_NULLPTR);
  ~PrimitiveMenu();

signals:
  void block();
  void sphere();
  void cylinder();
  void cone();
  void torus();
  void NUBSurface();
  void revolve();
  void line();
  void circle();
  void ellipse();
  void parabola();
  void hyperbola();
  void NUBcurve();
};

#endif // PRIMITIVIEMENU_HPP
