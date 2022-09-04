cmake_minimum_required(VERSION 3.5)
project(components)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Components/Pose.h
${PROJECT_SOURCE_DIR}/include/Components/Frame.h
${PROJECT_SOURCE_DIR}/include/Components/MapPoint.h
)

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_sources(${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${PROJECT_INCS}>)

target_include_directories(${PROJECT_NAME} INTERFACE
                          $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>)

target_link_libraries(${PROJECT_NAME} INTERFACE
                      ${OpenCV_LIBS}
                      spdlog::spdlog
                      Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib COMPONENT install-app
        LIBRARY DESTINATION lib COMPONENT install-app
        RUNTIME DESTINATION bin COMPONENT install-app)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/Components
        DESTINATION include/Components
        FILES_MATCHING PATTERN "*.h")

export(TARGETS ${PROJECT_NAME} NAMESPACE rgbd_slam:
      APPEND FILE ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake)

file(APPEND
    ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
"set(RGBDSLAM_LIBRARIES \${RGBDSLAM_LIBRARIES} ${PROJECT_NAME})\n")
