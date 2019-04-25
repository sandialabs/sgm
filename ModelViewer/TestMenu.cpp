#include "TestMenu.hpp"

TestMenu::TestMenu(QWidget *parent) :
        QMenu(parent)
{
    setTitle(tr("&Test"));

#ifdef VIEWER_WITH_GTEST
    addSeparator();

    QAction *gtest_action = addAction("&Run gtest", this, SIGNAL(gtest()));
    gtest_action->setShortcut(tr("Ctrl+G"));
#endif

    addSeparator();

    QAction *check_action = addAction("&Check", this, SIGNAL(check()));
    check_action->setShortcut(tr("Ctrl+C"));

    QAction *free_edges_action = addAction("&Free edges", this, SIGNAL(free_edges()));
    free_edges_action->setShortcut(tr("Ctrl+F"));
}
