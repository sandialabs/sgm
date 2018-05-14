#ifndef SGMPROPERTIESWIDGET_HPP
#define SGMPROPERTIESWIDGET_HPP

#include <QTreeWidget>


class SGMPropertiesWidget : public QTreeWidget
{
  Q_OBJECT

public:
  SGMPropertiesWidget(QWidget *parent=Q_NULLPTR);
  ~SGMPropertiesWidget();
};

#endif // SGMPROPERTIESWIDGET_HPP
