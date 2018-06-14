#include "SGMGraphicsWidget.hpp"

#include "vtkActor.h"
#include "vtkDataArray.h"
#include "vtkGenericOpenGLRenderWindow.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkCamera.h"
#include "vtkTriangle.h"
#include "vtkTriangleStrip.h"
#include "vtkCellData.h"
#include "vtkPointData.h"

#include "vtkDoubleArray.h"
#include "vtkSmartPointer.h"

//#include "GView.hpp"

class pGraphicsFace
{
public:
  vtkPoints* points;
  vtkCellArray* cells;
  vtkDataArray* normals;
  vtkPolyData* polyData;
  vtkPolyDataMapper* mapper;
  vtkActor* actor;

  pGraphicsFace() :
    points(vtkPoints::New()),
    cells(vtkCellArray::New()),
    normals(nullptr),
    polyData(vtkPolyData::New()),
    mapper(vtkPolyDataMapper::New()),
    actor(vtkActor::New())
  {
    polyData->SetPoints(points);
    polyData->SetPolys(cells);

    mapper->SetInputData(polyData);
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.5, 0.5, 1.0);
  }

  ~pGraphicsFace()
  {
    points->Delete();
    cells->Delete();
    polyData->Delete();
    mapper->Delete();
    actor->Delete();
  }
};

class pGraphicsEdge
{
public:
  vtkPoints* points;
  vtkCellArray* lines;
  vtkPolyData* polyData;
  vtkPolyDataMapper* mapper;
  vtkActor* actor;

  pGraphicsEdge() :
    points(vtkPoints::New()),
    lines(vtkCellArray::New()),
    polyData(vtkPolyData::New()),
    mapper(vtkPolyDataMapper::New()),
    actor(vtkActor::New())
  {
    polyData->SetPoints(points);
    polyData->SetLines(lines);
    mapper->SetInputData(polyData);
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
  }

  ~pGraphicsEdge()
  {
    points->Delete();
    lines->Delete();
    polyData->Delete();
    mapper->Delete();
    actor->Delete();
  }
};


struct pGraphicsData
{
  vtkRenderer* renderer;
  std::vector<pGraphicsFace*> mFaces;
  std::vector<pGraphicsEdge*> mEdges;
};



SGMGraphicsWidget::SGMGraphicsWidget(QWidget *parent, Qt::WindowFlags f) :
  QVTKOpenGLWidget(parent, f),
  dPtr(new pGraphicsData)
{
  vtkGenericOpenGLRenderWindow* myrenwin = vtkGenericOpenGLRenderWindow::New();
  this->SetRenderWindow(myrenwin);

  dPtr->renderer = vtkRenderer::New();
  dPtr->renderer->SetBackground(0.5, 0.5, 0.5);
  myrenwin->AddRenderer(dPtr->renderer);
  myrenwin->Delete();
}

SGMGraphicsWidget::~SGMGraphicsWidget()
{
  clear();
  dPtr->renderer->Delete();

  delete dPtr;
}

void SGMGraphicsWidget::clear()
{
  for(pGraphicsFace *face : dPtr->mFaces)
    delete face;

  for(pGraphicsEdge *edge : dPtr->mEdges)
    delete edge;

  dPtr->mFaces.clear();
  dPtr->mEdges.clear();
}

void SGMGraphicsWidget::remove_faces()
{
  dPtr->renderer->RemoveAllViewProps();
}

void SGMGraphicsWidget::add_face(const std::vector<SGM::Point3D>      &points,
                                 const std::vector<size_t>            &triangles,
                                 const std::vector<SGM::UnitVector3D> &norms)
{
  pGraphicsFace *face = new pGraphicsFace;
  dPtr->mFaces.push_back(face);

  // Add points and normals to the face

  vtkSmartPointer<vtkDoubleArray> normalsArray =
      vtkSmartPointer<vtkDoubleArray>::New();
  normalsArray->SetNumberOfComponents(3); //3d normals (ie x,y,z)
  normalsArray->SetNumberOfTuples(norms.size());

  face->polyData->GetPointData()->SetNormals(normalsArray);
  face->points->SetNumberOfPoints(points.size());

  for(size_t i=0; i<points.size(); i++)
      {
      SGM::UnitVector3D Norm=norms[i];
      normalsArray->SetTypedTuple(i, &Norm.m_x);
      SGM::Point3D point = points[i];
      face->points->SetPoint(i, point.m_x, point.m_y, point.m_z);
      }

  // set up triangles
  size_t num_tris = triangles.size()/3;
  for(size_t i=0; i<num_tris; i++)
      {
      face->cells->InsertNextCell(3);
      face->cells->InsertCellPoint(triangles[i*3]);
      face->cells->InsertCellPoint(triangles[i*3+1]);
      face->cells->InsertCellPoint(triangles[i*3+2]);
      }

  dPtr->renderer->AddActor(face->actor);
}

void SGMGraphicsWidget::add_edge(const std::vector<SGM::Point3D> &points)
{
  pGraphicsEdge *edge = new pGraphicsEdge;
  dPtr->mEdges.push_back(edge);

  // Add points to the edge
  size_t point_counter = 0;
  for(const SGM::Point3D &point : points)
    edge->points->InsertPoint(point_counter++, point.m_x, point.m_y, point.m_z);

  // Setup the lines
  edge->lines->InsertNextCell((int)points.size());
  for(size_t i = 0; i < points.size(); i++)
    edge->lines->InsertCellPoint(i);

  dPtr->renderer->AddActor(edge->actor);
}

void SGMGraphicsWidget::reset_view()
{
  dPtr->renderer->ResetCamera();
  vtkCamera *camera=dPtr->renderer->GetActiveCamera();
  camera->SetParallelProjection(1);
  this->renderVTK();
}
