#ifndef SGMTREEWIDGET_HPP
#define SGMTREEWIDGET_HPP

#include <QTreeWidget>
#include <set>

#include "SGMEntityClasses.h"

class SGMTreeWidget : public QTreeWidget
{
  Q_OBJECT

public:
  SGMTreeWidget(QWidget *parent=Q_NULLPTR);
  ~SGMTreeWidget();

  void add_entity(const SGM::Entity &ent,
                  const std::set<SGM::Entity> &parents,
                  const std::set<SGM::Entity> &children);

  void remove_entity(const SGM::Entity &ent);
};

#endif // SGMTREEWIDGET_HPP
