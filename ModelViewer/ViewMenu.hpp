#ifndef VIEWMENU_HPP
#define VIEWMENU_HPP

#include <QMenu>

class ViewMenu : public QMenu
{
  Q_OBJECT
public:
  ViewMenu(QWidget *parent=Q_NULLPTR);
  ~ViewMenu();

signals:

  void faces();
  void edges();
  void vertices();
  void facet();
  void uvspace();
  void perspective();
};

#endif // VIEWMENU_HPP
