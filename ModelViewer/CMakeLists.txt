project(ModelViewer)

cmake_minimum_required(VERSION 3.3)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)


#use lower warning level for viewer project
if(MSVC)
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W3" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
  endif()
  # Force to always compile with WX
  if(CMAKE_CXX_FLAGS MATCHES "/WX")
    string(REGEX REPLACE "/WX" "/WX-" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  endif()
endif(MSVC)


get_target_property(_loc Qt5::qmake LOCATION)
get_filename_component(_qmake_dir ${_loc} DIRECTORY)
get_filename_component(_qt_dir ${_qmake_dir} DIRECTORY)

option(BUILD_VIEWER_WITH_GTEST "Build viewer with gtest" ON)

set(MODEL_VIEWER_UIS
    MainWindow.ui)

qt5_wrap_ui(UI_SRC ${MODEL_VIEWER_UIS})

set(MODEL_VIEWER_SRC
    ${UI_SRC}
    ${SGM_VIEWER_GTEST_SOURCES}
    FileMenu.cpp
    FileMenu.hpp
    ViewMenu.cpp
    ViewMenu.hpp
    TestDialog.cpp
    TestDialog.hpp
    TestMenu.cpp
    TestMenu.hpp
    PrimitiveMenu.cpp
    PrimitiveMenu.hpp
    main.cpp
    MainWindow.cpp
    MainWindow.hpp
    ModelData.cpp
    ModelData.hpp
    SGMGraphicsWidget.cpp
    SGMGraphicsWidget.hpp
    SGMTreeWidget.cpp
    SGMTreeWidget.hpp
    )

add_executable(sgm_viewer ${MODEL_VIEWER_SRC})

target_link_libraries(sgm_viewer PRIVATE
  SGM
  Qt5::Widgets Qt5::OpenGL
  )

target_include_directories(sgm_viewer PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR})

if(BUILD_VIEWER_WITH_GTEST)
  message(STATUS "Build viewer with gtest")
  target_link_libraries(sgm_viewer PRIVATE gtest)
  target_compile_definitions(sgm_viewer PRIVATE -DVIEWER_WITH_GTEST)
endif()

if(APPLE)
  set(QT_CONF_OUTDIR $<TARGET_FILE_DIR:sgm_viewer>/../Resources)
else()
  set(QT_CONF_OUTDIR $<TARGET_FILE_DIR:sgm_viewer>)
endif()
file(GENERATE OUTPUT ${QT_CONF_OUTDIR}/qt.conf CONTENT "[Paths]\nPrefix = ${_qt_dir}")

