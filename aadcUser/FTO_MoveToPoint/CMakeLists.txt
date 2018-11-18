cmake_minimum_required(VERSION 3.10.0)

project(move_to_point)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LINK_DIRS})
include_directories(${Boost_INCLUDE_DIRS}) 


set (SOURCEFILES    	stdafx.h 
						${AADC_DIR}/include/tinyxml2.cpp
                        cMoveToPoint.h
                        cMoveToPoint.cpp
)




#target_link_libraries(line_filter ${OpenCV_LIBS})

adtf_add_filter(${PROJECT_NAME}
				${SOURCEFILES}
)

#find_package(OpenCV REQUIRED COMPONENTS core highgui)
#find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)
#install dlls for opencv to binary folder 
if(WIN32)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)
endif(WIN32)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 )



target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES})

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
