#ifndef VIEWMENU_HPP
#define VIEWMENU_HPP

#include <QMenu>

class ViewMenu : public QMenu
{
Q_OBJECT
public:
    explicit ViewMenu(QWidget *parent = Q_NULLPTR);

    ~ViewMenu() override = default;

signals:

    void faces();

    void edges();

    void vertices();

    void facet();

    void uvspace();

    void perspective();

    void background();
};

#endif // VIEWMENU_HPP
