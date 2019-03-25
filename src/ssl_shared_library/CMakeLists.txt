cmake_minimum_required(VERSION 2.8.3)
project(ssl_shared_library)
add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


if(APPLE)
	message("OS is APPLE")
	find_package(catkin REQUIRED COMPONENTS)
	find_package(Protobuf REQUIRED)
else()
	message("OS is linux (not apple)")
	set(PTHREAD_LIB "")
	find_package(Threads)
	find_package(catkin REQUIRED COMPONENTS
		# Threads
		)
	find_package(Protobuf REQUIRED)

endif()
# find_package(OpenGL REQUIRED)
# find_package(protobuf_ssl REQUIRED)
# find_package(robocup_client REQUIRED)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS net util
 LIBRARIES robocup_client_lib ${PTHREAD_LIB} #protobuf_ssl_lib
#  CATKIN_DEPENDS other_catkin_pkg
 # DEPENDS Protobuf
)


include(FindProtobuf)

set (PROTO_PATH proto)
set (ROBOCUP_CLIENT net)
set (SSL_VISION_UTIL util)


# message("CMAKE_HOME_DIRECTORY:" ${CMAKE_HOME_DIRECTORY})
# ### protocol buffer generation code ###
set(PROTO_FILES
    ${PROTO_PATH}/messages_robocup_ssl_detection.proto
    ${PROTO_PATH}/messages_robocup_ssl_geometry.proto
    ${PROTO_PATH}/messages_robocup_ssl_geometry_legacy.proto
    ${PROTO_PATH}/messages_robocup_ssl_refbox_log.proto
    ${PROTO_PATH}/messages_robocup_ssl_wrapper.proto
    ${PROTO_PATH}/messages_robocup_ssl_wrapper_legacy.proto
)

set(SSL_UTIL
    ${SSL_VISION_UTIL}/util.h
    ${SSL_VISION_UTIL}/timer.h
    ${SSL_VISION_UTIL}/geometry.h
    )

set(ROBOCUP_CLIENT_SOURCES
	${ROBOCUP_CLIENT}/netraw.cpp 
	${ROBOCUP_CLIENT}/robocup_ssl_client.cpp
	${ROBOCUP_CLIENT}/netraw.h
	${ROBOCUP_CLIENT}/robocup_ssl_client.h
)
# include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${PROTO_FILES})

include_directories (include 
	${PROTOBUF_INCLUDE_DIRS} 
	${catkin_INCLUDE_DIRS} 
	${SSL_VISION_UTIL}
	${PROTO_PATH} 
	${ROBOCUP_CLIENT}
	${CMAKE_CURRENT_BINARY_DIR}
	${Boost_INCLUDE_DIRS}
	)

# message("PROTO_HEADERS: " ${PROTO_HEADERS})
# message("PROTO_SOURCES: " ${PROTO_SOURCES})

# add_library(protobuf_ssl_lib SHARED 
# 	${PROTO_SOURCES} 
# 	${PROTO_HEADERS} 
# 	${PROTO_FILES}
# 	)

add_library(robocup_client_lib SHARED 
	${PROTO_SOURCES} 
	${PROTO_HEADERS} 
	${PROTO_FILES}
	${ROBOCUP_CLIENT_SOURCES}
	${SSL_UTIL} 
	)

# add_library(ssl_util_lib SHARED
# 	${SSL_UTIL}
# 	)

# target_link_libraries(protobuf_ssl_lib 
# 	${Protobuf_LIBRARY} 
# 	${CMAKE_THREAD_LIBS_INIT} 
# 	${catkin_LIBRARIES}
# 	)

target_link_libraries(robocup_client_lib 
	${catkin_LIBRARIES}
	${Protobuf_LIBRARY} 
	${Boost_LIBRARIES}
	)


## Mark executables and/or libraries for installation
install(
	TARGETS  robocup_client_lib #protobuf_ssl_lib
  	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Mark cpp header files for installation
install(DIRECTORY ${ROBOCUP_CLIENT} ${SSL_VISION_UTIL}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

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
