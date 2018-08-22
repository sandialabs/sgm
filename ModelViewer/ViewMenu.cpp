#include "ViewMenu.hpp"

ViewMenu::ViewMenu(QWidget *parent) :
        QMenu(parent)
{
    setTitle(tr("&View"));

    QAction *face_action = addAction("Show &Faces", this, SIGNAL(faces()));
    face_action->setShortcut(tr("Ctrl+F"));
    face_action->setCheckable(true);
    face_action->setChecked(true);

    QAction *edge_action = addAction("Show &Edges", this, SIGNAL(edges()));
    edge_action->setShortcut(tr("Ctrl+E"));
    edge_action->setCheckable(true);
    edge_action->setChecked(true);

    QAction *vertices_action = addAction("Show &Vertices", this, SIGNAL(vertices()));
    vertices_action->setShortcut(tr("Ctrl+V"));
    vertices_action->setCheckable(true);

    QAction *facet_action = addAction("Show Fa&cet", this, SIGNAL(facet()));
    facet_action->setShortcut(tr("Ctrl+f"));
    facet_action->setCheckable(true);

    addSeparator();

    QAction *urspace_action = addAction("&UV Space Mode", this, SIGNAL(uvspace()));
    urspace_action->setShortcut(tr("Ctrl+u"));
    urspace_action->setCheckable(true);

    QAction *perspective_action = addAction("&Perspective Mode", this, SIGNAL(perspective()));
    perspective_action->setShortcut(tr("Ctrl+p"));
    perspective_action->setCheckable(true);

    addSeparator();

    QAction *background_action = addAction("&Background Color", this, SIGNAL(background()));
    background_action->setShortcut(tr("Ctrl+B"));
}
