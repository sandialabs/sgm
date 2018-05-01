project(ModelViewer)

cmake_minimum_required(VERSION 3.3)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)
find_package(VTK)

include(${VTK_USE_FILE})

get_target_property(_loc Qt5::qmake LOCATION)
get_filename_component(_qmake_dir ${_loc} DIRECTORY)
get_filename_component(_qt_dir ${_qmake_dir} DIRECTORY)


set(MODEL_VIEWER_UIS
    MainWindow.ui)

qt5_wrap_ui(UI_SRC ${MODEL_VIEWER_UIS})

set(MODEL_VIEWER_SRC
    ${UI_SRC}
    main.cpp
    MainWindow.cpp
    MainWindow.hpp
    ModelData.cpp
    ModelData.hpp
    QVTKOpenGLWidget.cxx
    QVTKOpenGLWidget.h
    QVTKInteractor.cxx
    QVTKInteractor.h
    QVTKInteractorInternal.h
    QVTKInteractorAdapter.cxx
    QVTKInteractorAdapter.h
    SGMGraphicsWidget.cpp
    SGMGraphicsWidget.hpp
    SGMTreeWidget.cpp
    SGMTreeWidget.hpp
    )


add_executable(sgm_viewer ${MODEL_VIEWER_SRC})

target_link_libraries(sgm_viewer PRIVATE
  SGM
  Qt5::Widgets Qt5::OpenGL
  ${VTK_LIBRARIES}
  vtkInteractionStyle
  vtkRenderingOpenGL2)

foreach(vtk_inc ${VTK_INCLUDE_DIRS})
  target_include_directories(sgm_viewer PRIVATE $<BUILD_INTERFACE:${vtk_inc}>)
endforeach()

target_include_directories(sgm_viewer PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR})


if(APPLE)
  set(QT_CONF_OUTDIR $<TARGET_FILE_DIR:sgm_viewer>/../Resources)
else()
  set(QT_CONF_OUTDIR $<TARGET_FILE_DIR:sgm_viewer>)
endif()
file(GENERATE OUTPUT ${QT_CONF_OUTDIR}/qt.conf CONTENT "[Paths]\nPrefix = ${_qt_dir}")

