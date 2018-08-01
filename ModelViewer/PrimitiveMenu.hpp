#ifndef PRIMITIVIEMENU_HPP
#define PRIMITIVIEMENU_HPP

#include <QMenu>

class PrimitiveMenu : public QMenu
{
Q_OBJECT
public:
    explicit PrimitiveMenu(QWidget *parent = Q_NULLPTR);

    ~PrimitiveMenu() override = default;

signals:

    void block();

    void sphere();

    void cylinder();

    void cone();

    void torus();

    void revolve();

    void line();

    void circle();

    void ellipse();

    void parabola();

    void hyperbola();

    void NUBcurve();

    void TorusKnot();
};

#endif // PRIMITIVIEMENU_HPP
