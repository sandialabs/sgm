project(ModelViewer)

cmake_minimum_required(VERSION 3.3)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)


# When we drop VTK see if we can stop hiding this warning
if(MSVC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4996")
endif()

get_target_property(_loc Qt5::qmake LOCATION)
get_filename_component(_qmake_dir ${_loc} DIRECTORY)
get_filename_component(_qt_dir ${_qmake_dir} DIRECTORY)

option(BUILD_VIEWER_WITH_GTEST "Build viewer with gtest" ON)

set(MODEL_VIEWER_UIS
    MainWindow.ui)

if(BUILD_VIEWER_WITH_GTEST)
  get_filename_component(MODEL_VIEWER_TEST_DIR Tests ABSOLUTE BASE_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
  set(MODEL_VIEWER_TEST_SRC
          ${MODEL_VIEWER_TEST_DIR}/intersection_check.cpp
          ${MODEL_VIEWER_TEST_DIR}/volume_check.cpp
          ${MODEL_VIEWER_TEST_DIR}/test_utility.cpp)
else()
  set(MODEL_VIEWER_TEST_SRC "")
endif()

qt5_wrap_ui(UI_SRC ${MODEL_VIEWER_UIS})

set(MODEL_VIEWER_SRC
    ${UI_SRC}
    ${MODEL_VIEWER_TEST_SRC}
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
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
  target_link_libraries(sgm_viewer PRIVATE gtest)
  target_compile_definitions(sgm_viewer PRIVATE -DVIEWER_WITH_GTEST)
endif()

if(APPLE)
  set(QT_CONF_OUTDIR $<TARGET_FILE_DIR:sgm_viewer>/../Resources)
else()
  set(QT_CONF_OUTDIR $<TARGET_FILE_DIR:sgm_viewer>)
endif()
file(GENERATE OUTPUT ${QT_CONF_OUTDIR}/qt.conf CONTENT "[Paths]\nPrefix = ${_qt_dir}")

