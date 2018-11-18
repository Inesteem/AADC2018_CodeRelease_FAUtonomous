##############################################################################################
# ADTF Example for data triggered filters
# Use example as is! No additional support provided!
#
# (c) Audi Electronics Venture GmbH
##############################################################################################

cmake_minimum_required(VERSION 3.10.0)
#project(sync_filter)

#if (NOT TARGET adtf::filtersdk)
#    find_package(ADTF COMPONENTS filtersdk)
#endif()

#set (EXAMPLE_TARGETNAME demo_synchronizer_filter)
#set (EXAMPLE_CATEGORY   adtf/filters/standard_filters)
#set (EXAMPLE_SOURCES    synchronizer_filter.h
#                        synchronizer_filter.cpp)

##we use the special ADTF CMake command to add an shared object/DLL which has an extension called .adtfplugin
#adtf_add_filter(${EXAMPLE_TARGETNAME} ${EXAMPLE_SOURCES})

##install the program you need
#adtf_install_filter(${EXAMPLE_TARGETNAME} src/examples/bin)

##this is to add a folder within the ADTF examples solution
#set_property(TARGET ${EXAMPLE_TARGETNAME} PROPERTY FOLDER examples/${EXAMPLE_CATEGORY})

#adtf_create_plugindescription(
#    TARGET
#        ${EXAMPLE_TARGETNAME}
#    PLUGIN_SUBDIR
#        "src/examples/bin"
#)

project(SyncFilter)
include_directories(${Boost_INCLUDE_DIRS})
#include_directories(${OpenCV_INCLUDE_DIRS})

set (SOURCEFILES
                        synchronizer_filter.h
                        synchronizer_filter.cpp

)

adtf_add_filter(${PROJECT_NAME}
                                ${SOURCEFILES}
)

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 )
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES})
#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
