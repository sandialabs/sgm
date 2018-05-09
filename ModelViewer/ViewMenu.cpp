#include "ViewMenu.hpp"

#include <QSettings>

ViewMenu::ViewMenu(QWidget *parent) :
  QMenu(parent)
{
  setTitle(tr("&View"));

  QAction* zoom_action = addAction("&Zoom", this, SIGNAL(zoom()));
  zoom_action->setShortcut(tr("Ctrl+Z"));
  QAction* wire_action = addAction("&Wire Frame Mode", this, SIGNAL(wire()));
  wire_action->setShortcut(tr("Ctrl+W"));
  QAction* facet_action = addAction("&Facet Mode", this, SIGNAL(facet()));
  facet_action->setShortcut(tr("Ctrl+F"));
  QAction* urspace_action = addAction("&UV Space Mode", this, SIGNAL(uvspace()));
  urspace_action->setShortcut(tr("Ctrl+u"));
  QAction* perspective_action = addAction("&Perspective Mode", this, SIGNAL(perspective()));
  perspective_action->setShortcut(tr("Ctrl+p"));
}

ViewMenu::~ViewMenu()
{}
