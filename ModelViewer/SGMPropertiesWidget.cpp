#include "SGMPropertiesWidget.hpp"
#include <QTreeWidgetItem>


SGMPropertiesWidget::SGMPropertiesWidget(QWidget *parent) :
        QTreeWidget(parent)
{
    // connect to tree's selection changed signal, or something triggered by that signal

}

// what data is available for any given entity
// Is it identified by its id?
// Is id space shared by all entities?
