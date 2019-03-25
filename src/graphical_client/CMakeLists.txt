cmake_minimum_required(VERSION 2.8.3)
project(graphical_client)

add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Qt5Core)
if (Qt5Core_FOUND)
	find_package(Qt5 COMPONENTS Core Widgets OpenGL REQUIRED)
else()
	find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL  REQUIRED)
endif(Qt5Core_FOUND)

find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs 
	geometry_msgs 
	ssl_shared_library
	message_generation
	)

# find_package(protobuf_ssl_lib REQUIRED)
# find_package(robocup_client_lib REQUIRED)
find_package(OpenGL REQUIRED)

# message("INSTALL PATH:" ${CATKIN_PACKAGE_LIB_DESTINATION})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
 	FILES
 	Pose2D_Array.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	# std_msgs
  	geometry_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
 # LIBRARIES
#  CATKIN_DEPENDS other_catkin_pkg
 	CATKIN_DEPENDS  ssl_shared_library #robocup_client_lib
 # DEPENDS  protobuf_ssl_lib robocup_client_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}

	)
#include( ${PROJECT_BINARY_DIR} )

# include(FindProtobuf)

# set (PROTO_PATH ${CMAKE_HOME_DIRECTORY}/shared/proto)
# set (ROBOCUP_CLIENT ${CMAKE_HOME_DIRECTORY}/shared/net)
# set (SSL_VISION_UTIL ${CMAKE_HOME_DIRECTORY}/shared/util)

# find_package(Protobuf REQUIRED)
# include_directories ( include ${PROTOBUF_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${SSL_VISION_UTIL} ${PROTO_PATH} ${ROBOCUP_CLIENT})

# message("CMAKE_HOME_DIRECTORY:" ${CMAKE_HOME_DIRECTORY})
# ### protocol buffer generation code ###
# set(PROTO_FILES
#     ${PROTO_PATH}/messages_robocup_ssl_detection.proto
#     ${PROTO_PATH}/messages_robocup_ssl_geometry.proto
#     ${PROTO_PATH}/messages_robocup_ssl_geometry_legacy.proto
#     ${PROTO_PATH}/messages_robocup_ssl_refbox_log.proto
#     ${PROTO_PATH}/messages_robocup_ssl_wrapper.proto
#     ${PROTO_PATH}/messages_robocup_ssl_wrapper_legacy.proto
# )

# set(SSL_UTIL
#     ${SSL_VISION_UTIL}/util.h
#     ${SSL_VISION_UTIL}/timer.h
#     ${SSL_VISION_UTIL}/geometry.h
#     )

# set(ROBOCUP_CLIENT_SOURCES
# 	${ROBOCUP_CLIENT}/netraw.cpp 
# 	${ROBOCUP_CLIENT}/robocup_ssl_client.cpp
# 	${ROBOCUP_CLIENT}/netraw.h
# 	${ROBOCUP_CLIENT}/robocup_ssl_client.h
# )
# include_directories(${CMAKE_CURRENT_BINARY_DIR})
# protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${PROTO_FILES})


# add_library(protobuf_ssl_lib SHARED ${PROTO_SOURCES} ${PROTO_HEADERS} ${PROTO_FILES})

# add_library(robocup_client_lib SHARED ${SSL_UTIL} ${ROBOCUP_CLIENT_SOURCES}  )

# target_link_libraries(protobuf_ssl_lib ${Protobuf_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})

# target_link_libraries(robocup_client_lib ${catkin_LIBRARIES})

# include_directories(${CMAKE_CURRENT_BINARY_DIR})
# protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${PROTO_FILES})
# include_directories(${CMAKE_BINARY_DIR}/vision_receiver/)
# add_library(protobuf ${PROTO_SOURCES} ${PROTO_HEADERS} ${PROTO_FILES})

# target_link_libraries(protobuf ${Protobuf_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})


# set (PROTO_PATH ${CMAKE_HOME_DIRECTORY}/shared/proto)
# set (SSL_VISION_UTIL ${CMAKE_HOME_DIRECTORY}/shared/util)

# include_directories (include ${catkin_INCLUDE_DIRS} ${SSL_VISION_UTIL})

# set (PROTO_PATH /opt/ssl-vision/src/shared/proto)
# set (SSL_VISION_UTIL /opt/ssl-vision/src/shared/util/)


# ### protocol buffer generation code ###
# set(SSL_SOURCES
#     ${SSL_VISION_UTIL}/timer.h
#     ${SSL_VISION_UTIL}/geometry.h
#     ${SSL_VISION_UTIL}/util.h
# )
# include_directories ( include ${catkin_INCLUDE_DIRS} ../../shared/util ../../shared/misc)
# add_library(ssl_lib ${SSL_SOURCES})
# set_target_properties(ssl_lib PROPERTIES LINKER_LANGUAGE CXX)


set (QT_USE_QTOPENGL true)

## generate moc files for graphicalClient
if (Qt5Core_FOUND)
	# find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

	qt5_wrap_cpp(GCLIENT_MOC_SRCS include/soccerview.h)
	set(qtlibs_aux "Qt5::Widgets")
else()
	# find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL  REQUIRED)
	include ( ${QT_USE_FILE} )
	qt4_wrap_cpp(GCLIENT_MOC_SRCS include/soccerview.h)
	set(qtlibs_aux ${QT_LIBRARIES})

endif(Qt5Core_FOUND)


# add_library(soccer_files soccerview.h)
# set_target_properties(soccer_files PROPERTIES LINKER_LANGUAGE CXX)
# target_link_libraries(soccer_files ${ssl_lib})

#include ( ${QT_USE_FILE} )

## Declare a cpp library
# add_library(graphical_client
#   src/${PROJECT_NAME}/graphical_client.cpp
# )

#add_library(graphical_client  
#)

#add_library(graphical_client_libs
#  ../../shared/util/timer.h
#)

## Declare a cpp executable
# add_executable(graphical_client_node src/graphical_client_node.cpp)




add_executable(graphical_client_node 
	${GCLIENT_MOC_SRCS}
  	soccerview.cpp
  	graphical_client_node.cpp
  	gltext.cpp
)

# if ( APPLE )

# 	target_link_libraries(graphical_client_node
# 		${catkin_LIBRARIES}
# 		${qtlibs_aux}
# 		${OPENGL_gl_LIBRARY}
# 		${OPENGL_glu_LIBRARY}
# 		)
# 	qt5_use_modules(graphical_client_node Widgets OpenGL)
# else()

	target_link_libraries(graphical_client_node
		${catkin_LIBRARIES}
		${qtlibs_aux}
		${Boost_LIBRARIES}
		${OPENGL_gl_LIBRARY}
		${OPENGL_glu_LIBRARY}
		)
	qt5_use_modules(graphical_client_node Widgets OpenGL)

# endif()
#target_link_libraries(graphical_client_node graphical_client_libs ${catkin_LIBRARIES} )


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(graphical_client_node graphical_client_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(graphical_client_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS  protobuf_ssl_lib robocup_client_lib
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_graphical_client.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
