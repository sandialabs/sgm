#include "SGMTreeWidget.hpp"

#include <QTreeWidgetItem>
#include <QMouseEvent>
#include <QMenu>

#include "qcolordialog.h"

SGMTreeWidget::SGMTreeWidget(QWidget *parent) :
    QTreeWidget(parent)
    {
    }


void SGMTreeWidget::add_entity(const SGM::Entity &,//ent,
                               const std::set<SGM::Entity> &,//parents,
                               const std::set<SGM::Entity> &)//children)
    {
    }

void SGMTreeWidget::remove_entity(const SGM::Entity &)//ent)
    {
    }

void SGMTreeWidget::mouseReleaseEvent(QMouseEvent* event)
    {
    if(event->button() == Qt::RightButton)
        {
        std::vector<SGM::Entity> aEnts;
        size_t nEnts=mModel->GetSelection(aEnts);
        if(nEnts)
            {
            QMenu menu;
            QAction* option_color = menu.addAction(tr("Set Color"));
            QAction* option_remove_color = menu.addAction(tr("Remove Color"));
            QAction* option_copy = menu.addAction(tr("Copy"));
            QAction* option_delete = menu.addAction(tr("Delete"));
            QAction* option_unhook = menu.addAction(tr("Unhook"));
            QAction* option_rebuild = menu.addAction(tr("Rebuild Tree"));
            QAction* result = menu.exec(QCursor::pos());
            mModel->ClearSelection();
            size_t Index1;
            if(result == option_color)
                {
                QColor color = QColorDialog::getColor(Qt::yellow, this );
                for(Index1=0;Index1<nEnts;++Index1)
                    {
                    mModel->ChangeColor(aEnts[Index1],color.red(),color.green(),color.blue());
                    }
                }
            else if(result == option_remove_color)
                {
                for(Index1=0;Index1<nEnts;++Index1)
                    {
                    mModel->RemoveColor(aEnts[Index1]);
                    }
                }
            else if(result == option_copy)
                {
                for(Index1=0;Index1<nEnts;++Index1)
                    {
                    mModel->Copy(aEnts[Index1]);
                    }
                }
            else if(result == option_unhook)
                {
                mModel->Unhook(aEnts);
                }
            else if(result == option_delete)
                {
                for(Index1=0;Index1<nEnts;++Index1)
                    {
                    mModel->DeleteEntity(aEnts[Index1]);
                    }
                mModel->rebuild_graphics();
                }
            else if(result == option_rebuild)
                {
                mModel->rebuild_tree();
                }
            }
        else
            {
            QMenu menu;
            QAction* option_rebuild = menu.addAction(tr("Rebuild Tree"));
            QAction* result = menu.exec(QCursor::pos());
            if(result == option_rebuild)
                {
                mModel->rebuild_tree();
                }
            }
        }
    else
        {
        QTreeWidget::mouseReleaseEvent(event);
        }
    }