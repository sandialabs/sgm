#include "SGMTreeWidget.hpp"

#include "SGMEntityFunctions.h"

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

bool AllFaces(SGM::Result                    &rResult,
              std::vector<SGM::Entity> const &aEnts)
    {
    for(auto ent : aEnts)
        {
        if(SGM::GetType(rResult,ent)!=SGM::FaceType)
            {
            return false;
            }
        }
    return true;
    }

bool AllComplexes(SGM::Result                    &rResult,
                  std::vector<SGM::Entity> const &aEnts)
    {
    for(auto ent : aEnts)
        {
        if(SGM::GetType(rResult,ent)!=SGM::ComplexType)
            {
            return false;
            }
        }
    return true;
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
            QAction *option_color=nullptr,
                    *option_remove_color=nullptr,
                    *option_copy=nullptr,
                    *option_delete=nullptr,
                    *option_unhook=nullptr,
                    *option_cover=nullptr,
                    *option_rebuild=nullptr,
                    *option_boundary=nullptr,
                    *option_find_components=nullptr,
                    *option_merge=nullptr;

            option_color = menu.addAction(tr("Set Color"));
            option_remove_color = menu.addAction(tr("Remove Color"));
            option_copy = menu.addAction(tr("Copy"));
            option_delete = menu.addAction(tr("Delete"));
            SGM::Result rResult=mModel->GetResult();
            if(AllFaces(rResult,aEnts))
                {
                option_unhook = menu.addAction(tr("Unhook"));
                }
            if(AllComplexes(rResult,aEnts))
                {
                option_cover = menu.addAction(tr("Cover"));
                option_merge = menu.addAction(tr("Merge"));
                option_boundary = menu.addAction(tr("Boundary"));
                option_find_components = menu.addAction(tr("Split by Components"));
                }
            option_rebuild = menu.addAction(tr("Rebuild Tree"));

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
            else if(result == option_cover)
                {
                for(Index1=0;Index1<nEnts;++Index1)
                    {
                    mModel->Cover(aEnts[Index1]);
                    }
                }
            else if(result == option_merge)
                {
                for(Index1=0;Index1<nEnts;++Index1)
                    {
                    mModel->Merge(aEnts[Index1]);
                    }
                }
            else if(result == option_find_components)
                {
                for(Index1=0;Index1<nEnts;++Index1)
                    {
                    mModel->FindComponents(aEnts[Index1]);
                    }
                }
            else if(result == option_boundary)
                {
                for(Index1=0;Index1<nEnts;++Index1)
                    {
                    mModel->Boundary(aEnts[Index1]);
                    }
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