cmake_minimum_required(VERSION 3.5)
project(room_rgbdslam)


set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Feature.cpp
${PROJECT_SOURCE_DIR}/src/Odometry.cpp
${PROJECT_SOURCE_DIR}/src/ICP.cpp
${PROJECT_SOURCE_DIR}/src/Motion.cpp
${PROJECT_SOURCE_DIR}/src/BundleAdjustment.cpp
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Room/RGBDSlam/Feature.h
${PROJECT_SOURCE_DIR}/include/Room/RGBDSlam/Odometry.h
${PROJECT_SOURCE_DIR}/include/Room/RGBDSlam/ICP.h
${PROJECT_SOURCE_DIR}/include/Room/RGBDSlam/Motion.h
${PROJECT_SOURCE_DIR}/include/Room/RGBDSlam/BundleAdjustment.h
)

source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(g2o REQUIRED)

set(G2O_LIBRARIES
    g2o::csparse
    g2o::freeglut_minimal
    g2o::stuff
    g2o::opengl_helper
    g2o::core
    g2o::g2o_cli_library
    g2o::g2o_hierarchical_library
    g2o::g2o_simulator_library
    g2o::types_data
    g2o::types_slam2d
    g2o::types_slam3d
    g2o::types_sba
    g2o::types_sim3
    g2o::types_icp
    g2o::types_sclam2d
    g2o::types_slam2d_addons
    g2o::types_slam3d_addons
    g2o::solver_eigen)

add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})

target_include_directories(${PROJECT_NAME} PUBLIC
                          $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>)

target_link_libraries(${PROJECT_NAME}
                      ${OpenCV_LIBS}
                      spdlog::spdlog
                      ceres
                      g2o::types_sba
                      g2o::solver_eigen
                      room_components)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib COMPONENT install-app
        LIBRARY DESTINATION lib COMPONENT install-app
        RUNTIME DESTINATION bin COMPONENT install-app)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/Room/RGBDSlam
        DESTINATION include/Room/RGBDSlam
        FILES_MATCHING PATTERN "*.h")

export(TARGETS ${PROJECT_NAME} NAMESPACE room::
      APPEND FILE ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake)

file(APPEND
    ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
"set(ROOM_RGBDSLAM_LIBRARIES \${ROOM_RGBDSLAM_LIBRARIES} ${PROJECT_NAME})\n")
