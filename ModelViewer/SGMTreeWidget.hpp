#ifndef SGMTREEWIDGET_HPP
#define SGMTREEWIDGET_HPP

#include "SGMEntityClasses.h"

#include <QTreeWidget>
#include <set>

#include "SGMEntityClasses.h"

#include "ModelData.hpp"

class SGMTreeWidget : public QTreeWidget
{
  Q_OBJECT

public:

    explicit SGMTreeWidget(QWidget *parent=Q_NULLPTR);

    ~SGMTreeWidget() override = default;

    void add_entity(const SGM::Entity &ent,
                    const std::set<SGM::Entity> &parents,
                    const std::set<SGM::Entity> &children);

    void remove_entity(const SGM::Entity &ent);

    std::map<QTreeWidgetItem *,SGM::Entity> mTreeMap;

    void mouseReleaseEvent(QMouseEvent *event) override;

    ModelData *mModel;

    SGM::Entity mCurentEnt;

};

#endif // SGMTREEWIDGET_HPP
