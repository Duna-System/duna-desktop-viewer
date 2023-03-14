cmake_minimum_required(VERSION 3.5)

project(duna_viewer VERSION 0.2 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE DEBUG) #this enables specific logging

# On windows, install these packages with vcpkg. Notice CONFIG qualifier.

find_package(Qt5  COMPONENTS Core Gui OpenGL Widgets REQUIRED)
find_package(PCL 1.10 CONFIG REQUIRED COMPONENTS common io search)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(duna_optimizer REQUIRED)

add_definitions(${PCL_DEFINITIONS})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

#common source
include_directories(common)
set(common_src
	common/src/camera.cpp
	common/src/gl_text2d.cpp
	common/src/gldata.cpp
	common/src/gltext.cpp
	# QT MOCs
	common/common/camera.h
	common/common/gl_text2d.h
	common/common/gltext.h
	)
# 

#duna source
include_directories(include) # duna header files
set(duna_src
	src/main.cpp
	src/calibration_window.cpp
	src/calibration_window.ui
	src/main_window.cpp
	src/main_window.ui
	src/qgl_viewer.cpp
	src/image_viewer.cpp
	src/point_picker.cpp
	src/image_rectification.cpp
	src/cloud_texture.cpp
	src/utilities.cpp
	src/cloud_loader.cpp

	# QT MOCs
	include/calibration_window.h 
	include/qgl_viewer.h 
	include/image_viewer.h
	include/main_window.h 
)

#Copy shaders to bin folder
file(GLOB shaders RELATIVE ${CMAKE_SOURCE_DIR} "src/shaders/*")
foreach(shader ${shaders})
    file(COPY ${shader} DESTINATION ${CMAKE_BINARY_DIR}/bin/shaders) # build/bin/shaders
endforeach()

get_filename_component(ShadersAbsPath ${CMAKE_BINARY_DIR}/bin/shaders REALPATH)

message (STATUS "Shaders will be installed in " ${ShadersAbsPath}) # TODO check if it works in other systems

# default shader path
add_definitions (-DSHADER_PATH="${ShadersAbsPath}/")

# Main executable
add_executable(${PROJECT_NAME} ${duna_src} ${common_src} ) 
target_include_directories(${PROJECT_NAME} PUBLIC duna-optimizer/include)
target_link_libraries(${PROJECT_NAME} duna::duna_optimizer Qt5::Core Qt5::Widgets Qt5::Gui Qt5::OpenGL ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} ${OpenCV_LIBS} )

install (TARGETS
${PROJECT_NAME}
)


if(WIN32)
	# Try exporting all dependent libraries
	if(CMAKE_MINOR_VERSION GREATER_EQUAL 23 AND CMAKE_PATCH_VERSION GREATER_EQUAL 1)
		install(IMPORTED_RUNTIME_ARTIFACTS 
			Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL duna_optimizer pcl_common pcl_io 
			RUNTIME DESTINATION bin
			LIBRARY DESTINATION lib
			DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
		set(CPACK_GENERATOR NSIS)
		# include (InstallRequiredSystemLibraries)
		include(CPack)
	endif()
elseif(UNIX)
	# Try exporting all dependent libraries
	if(CMAKE_MINOR_VERSION GREATER_EQUAL 23 AND CMAKE_PATCH_VERSION GREATER_EQUAL 1)
		message(STATUS "Using 'IMPORTED_RUNTIME_ARTIFACTS' ")
		install(IMPORTED_RUNTIME_ARTIFACTS 
			Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL duna_optimizer
			)
		set(CPACK_GENERATOR DEB)
		set(CPACK_DEBIAN_PACKAGE_NAME "duna-viewer")
		set(CPACK_DEBIAN_PACKAGE_DEPENDS "libeigen3-dev,libpcl-common1.10,libpcl-io1.10,libdouble-conversion3,libpcre2-16-0, libyaml-cpp-dev")
		set(CPACK_PACKAGE_CONTACT "marcus.forte@dunasystem.com")
		include(CPack)
	endif()
endif()

