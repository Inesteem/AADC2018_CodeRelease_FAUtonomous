cmake_minimum_required(VERSION 3.10.0)

project(pid_graph_plot)

if (NOT TARGET adtf::ui)
    find_package(ADTF COMPONENTS filtersdk ui)
endif()

find_package(adtfui REQUIRED)

#define new targets for qt automoc generated projects (since cmake 3.9 autogen projects are created separately)
set_property(GLOBAL PROPERTY AUTOGEN_TARGETS_FOLDER AutoMoc)

adtf_use_qt(Widgets Gui)

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

set (HEADERS    stdafx.h
                GraphPlotFilter.h
                GraphPlotWidget.h
                FrameCounter.h
					  )
set (SOURCES
              GraphPlotFilter.cpp
              GraphPlotWidget.cpp
              FrameCounter.cpp
					  )


# Instruct CMake to run moc automatically when needed.
set (CMAKE_AUTOMOC ON)
set (CMAKE_AUTOUIC ON)
set (CMAKE_AUTORCC ON)

#uncomment if CMAKE_AUTOMOC/UIC is disabled
#ui/MainWindow.ui 
set (UI_SOURCE pid_graph_plot.ui)

adtf_add_filter(${PROJECT_NAME}
				${HEADERS}
				${UI_SOURCE}
				${SOURCES}
				)


target_link_libraries(${PROJECT_NAME} PUBLIC adtf::ui Qt5::Core Qt5::Gui Qt5::Widgets ${Boost_LIBRARIES} )
target_include_directories(${PROJECT_NAME} PUBLIC .)

#install the program you need    
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
    DEPENDENT_PLUGINS
        "adtf_xsystem" 
)

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
