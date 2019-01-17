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
    FileMenu.cpp
    FileMenu.hpp
    ViewMenu.cpp
    ViewMenu.hpp
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
  target_sources(sgm_viewer PRIVATE ${SGM_VIEWER_GTEST_SOURCES} TestDialog.cpp TestDialog.hpp)
  target_link_libraries(sgm_viewer PRIVATE gtest)
  target_compile_definitions(sgm_viewer PRIVATE -DVIEWER_WITH_GTEST)
endif()

if(APPLE)
  set(QT_CONF_OUTDIR $<TARGET_FILE_DIR:sgm_viewer>/../Resources)
else()
  set(QT_CONF_OUTDIR $<TARGET_FILE_DIR:sgm_viewer>)
endif()
file(GENERATE OUTPUT ${QT_CONF_OUTDIR}/qt.conf CONTENT "[Paths]\nPrefix = ${_qt_dir}")

install(TARGETS sgm_viewer EXPORT SGM
        RUNTIME DESTINATION bin COMPONENT Runtime
        LIBRARY DESTINATION lib COMPONENT Runtime
        ARCHIVE DESTINATION lib COMPONENT Development)


# Set some Qt variables that no longer exist for Qt5, so that we can use them like normal
# in packaging
get_target_property(QT_QTCORE_LIBRARY_RELEASE Qt5::Core IMPORTED_LOCATION_RELEASE)
  
set(qt_type)
include(GetPrerequisites)
gp_resolved_file_type("${QT_QTCORE_LIBRARY_RELEASE}" "${QT_QTCORE_LIBRARY_RELEASE}" "" "" qt_type)

if(NOT "${qt_type}" STREQUAL "system")

  set(QtLibs Bluetooth CLucene Concurrent Core DBus Designer DesignerComponents
    Gui Help Multimedia MultimediaQuick_p MultimediaWidgets Network Nfc
    OpenGL Positioning PrintSupport Qml Quick QuickParticles QuickTest
    QuickWidgets Sensors SerialPort Sql Svg Test WebChannel WebSockets
    Widgets Xml XmlPatterns)
  
  if(WIN32)
    set(QtLibs ${QtLibs} WinExtras)
  elseif(APPLE)
    set(QtLibs ${QtLibs} UiPlugin MacExtras)
  else()
    set(QtLibs ${QtLibs} EglDeviceIntegration X11Extras XcbQpa)
  endif()
  
  foreach(QtLib ${QtLibs})
    if(APPLE)
      if(EXISTS ${_qt_dir}/lib/Qt${QtLib}.framework/Versions/5/Qt${QtLib})
        install(PROGRAMS ${_qt_dir}/lib/Qt${QtLib}.framework/Versions/5/Qt${QtLib}
                DESTINATION bin/Qt${QtLib}.framework/Versions/5
                COMPONENT Runtime
                )
        if(EXISTS ${_qt_dir}/lib/Qt${QtLib}.framework/Versions/5/Resources)
          install(DIRECTORY ${_qt_dir}/lib/Qt${QtLib}.framework/Versions/5/Resources
                  DESTINATION bin/Qt${QtLib}.framework
                  COMPONENT Runtime
                  )
        endif()
      elseif(EXISTS ${_qt_dir}/lib/libQt${QtLib}.5.dylib)
        install(CODE
                "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E copy \"${_qt_dir}/lib/libQt${QtLib}.5.dylib\"
                  \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin\")
                execute_process(COMMAND chmod 755 \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin/libQt${QtLib}.5.dylib\")
                message(\"copying ${_qt_dir}/lib/libQt${QtLib}.5.dylib\")"
                COMPONENT Runtime
                )
      endif()
    elseif(WIN32)
      if(EXISTS ${_qt_dir}/bin/Qt5${QtLib}.dll)
        install(CODE
                "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E copy \"${_qt_dir}/bin/Qt5${QtLib}.dll\" \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin\")
                message(\"copying ${_qt_dir}/bin/Qt5${QtLib}.dll\")"
                COMPONENT Runtime
                )
      endif()
    else() # Linux
      if(EXISTS ${_qt_dir}/lib/libQt5${QtLib}.so.5)
        install(CODE
                "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E copy \"${_qt_dir}/lib/libQt5${QtLib}.so.5\" \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin\")
                execute_process(COMMAND chmod 755 \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin/libQt5${QtLib}.so.5\")
                message(\"copying ${_qt_dir}/lib/libQt5${QtLib}.so.5\")"
                COMPONENT Runtime
                )
      endif()
    endif()
  endforeach()

  set(QT_LIBRARY_DIR ${_qt_dir}/lib)
  set(QT_BINARY_DIR ${_qmake_dir})
  set(QT_PLUGINS_DIR ${_qt_dir}/plugins)
  set(QT_QTCORE_INCLUDE_DIR ${Qt5Core_INCLUDE_DIRS})


  # install Qt plugins
  SET(qt_plugin_dir bin)
  set(exclude_regex "(compat|debug|d4.dll|designer|exp|ilk|pdb|\\\\.svn)")
  if(CMAKE_SYSTEM_NAME MATCHES Linux)
    set(exclude_regex "(compat|debug|d4.dll|designer|exp|ilk|pdb|bearer|\\\\.svn)")
  endif()

  install(CODE
  "
    file(GLOB_RECURSE QT_PLUGINS \"${QT_PLUGINS_DIR}/*\")
    foreach(QT_FILE \${QT_PLUGINS})
      if(NOT QT_FILE MATCHES \"${exclude_regex}\")
        # Get the plugin's containing directory as a relative path
        get_filename_component(_plugin_dir \${QT_FILE} DIRECTORY)
        file(RELATIVE_PATH _plugin_dir \"${QT_PLUGINS_DIR}\" \${_plugin_dir})

        # Copy the file
        get_filename_component(name \"\${QT_FILE}\" NAME)
        set (result \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${qt_plugin_dir}/plugins/\${_plugin_dir}/\${name}\")
        message(STATUS \"copying \${QT_FILE} to \${result}\")
        execute_process(COMMAND ${CMAKE_COMMAND} -E copy \"\${QT_FILE}\" \"\${result}\")
        if(UNIX)
          execute_process(COMMAND chmod 755 \"\${result}\")
        endif()
      endif()
    endforeach()
  " COMPONENT Runtime)

  # install qt.conf file
  install(CODE "
  file(WRITE \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin/qt.conf\" \"\")
  " COMPONENT Runtime)
endif()
