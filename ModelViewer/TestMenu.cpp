#include "TestMenu.hpp"

TestMenu::TestMenu(QWidget *parent) :
        QMenu(parent)
{
    setTitle(tr("&Test"));

    QAction *all_action = addAction("&Run All...", this, SIGNAL(all()));
    all_action->setShortcut(tr("Ctrl+A"));
    QAction *number_action = addAction("&Run Number", this, SIGNAL(number()));
    number_action->setShortcut(tr("Ctrl+N"));
    QAction *script_action = addAction("&Run Script", this, SIGNAL(script()));
    script_action->setShortcut(tr("Ctrl+S"));

#ifdef VIEWER_WITH_GTEST
    addSeparator();

    QAction *gtest_action = addAction("&Run gtest", this, SIGNAL(gtest()));
    gtest_action->setShortcut(tr("Ctrl+G"));
#endif

    addSeparator();

    QAction *check_action = addAction("&Check", this, SIGNAL(check()));
    check_action->setShortcut(tr("Ctrl+C"));
}
