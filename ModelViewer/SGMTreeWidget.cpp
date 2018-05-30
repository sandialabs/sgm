#include "SGMTreeWidget.hpp"

#include <QTreeWidgetItem>

SGMTreeWidget::SGMTreeWidget(QWidget *parent) :
  QTreeWidget(parent)
{}

SGMTreeWidget::~SGMTreeWidget()
{}


void SGMTreeWidget::add_entity(const SGM::Entity &,//ent,
                               const std::set<SGM::Entity> &,//parents,
                               const std::set<SGM::Entity> &)//children)
{

}

void SGMTreeWidget::remove_entity(const SGM::Entity &)//ent)
{

}
