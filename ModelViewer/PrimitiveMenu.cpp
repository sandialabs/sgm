#include "PrimitiveMenu.hpp"

PrimitiveMenu::PrimitiveMenu(QWidget *parent) :
        QMenu(parent)
{
    setTitle(tr("&Primitive"));

    QAction *block_action = addAction("&Block", this, SIGNAL(block()));
    block_action->setShortcut(tr("Ctrl+B"));
    QAction *sphere_action = addAction("&Sphere", this, SIGNAL(sphere()));
    sphere_action->setShortcut(tr("Ctrl+S"));
    QAction *cylinder_action = addAction("&Cylinder", this, SIGNAL(cylinder()));
    cylinder_action->setShortcut(tr("Ctrl+C"));
    QAction *cone_action = addAction("C&one", this, SIGNAL(cone()));
    cone_action->setShortcut(tr("Ctrl+o"));
    QAction *torus_action = addAction("&Torus", this, SIGNAL(torus()));
    torus_action->setShortcut(tr("Ctrl+T"));
    QAction *revolve_action = addAction("Re&volve", this, SIGNAL(revolve()));
    revolve_action->setShortcut(tr("Ctrl+v"));

    addSeparator();

    QAction *line_action = addAction("&Line", this, SIGNAL(line()));
    line_action->setShortcut(tr("Ctrl+L"));
    QAction *circle_action = addAction("Cir&cle", this, SIGNAL(circle()));
    circle_action->setShortcut(tr("Ctrl+c"));
    QAction *ellipse_action = addAction("&Ellipse", this, SIGNAL(ellipse()));
    ellipse_action->setShortcut(tr("Ctrl+E"));
    QAction *parabola_action = addAction("&Parabola", this, SIGNAL(parabola()));
    parabola_action->setShortcut(tr("Ctrl+P"));
    QAction *hyperbola_action = addAction("&Hyperbola", this, SIGNAL(hyperbola()));
    hyperbola_action->setShortcut(tr("Ctrl+H"));
    QAction *nub_curve_action = addAction("NUB Cu&rve", this, SIGNAL(NUBcurve()));
    nub_curve_action->setShortcut(tr("Ctrl+r"));
    QAction *torus_knot_action = addAction("Torus &Knot", this, SIGNAL(TorusKnot()));
    torus_knot_action->setShortcut(tr("Ctrl+K"));
    QAction *complex_action = addAction("&Complex", this, SIGNAL(Complex()));
    complex_action->setShortcut(tr("Ctrl+C"));
}

