#ifndef SGMPROPERTIESWIDGET_HPP
#define SGMPROPERTIESWIDGET_HPP

#include <QTreeWidget>


class SGMPropertiesWidget : public QTreeWidget
{
Q_OBJECT

public:
    explicit SGMPropertiesWidget(QWidget *parent = Q_NULLPTR);

    ~SGMPropertiesWidget() override = default;
};

#endif // SGMPROPERTIESWIDGET_HPP
