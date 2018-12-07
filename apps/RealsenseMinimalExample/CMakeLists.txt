cmake_minimum_required(VERSION 3.11)

project(RealsenseMinimalExample)

find_package(realsense2 REQUIRED)

set(PROJECT_SRCS ${PROJECT_SOURCE_DIR}/src/RealsenseMinimalExample.cpp)

source_group("src" FILES ${PROJECT_SRCS})

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} PUBLIC realsense2::realsense2)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin COMPONENT install-app)
