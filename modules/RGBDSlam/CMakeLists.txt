cmake_minimum_required(VERSION 3.5)
project(rgbdslam)


set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Feature.cpp
${PROJECT_SOURCE_DIR}/src/Track.cpp
# ${PROJECT_SOURCE_DIR}/src/Odometry.cpp
# ${PROJECT_SOURCE_DIR}/src/ICP.cpp
# ${PROJECT_SOURCE_DIR}/src/Motion.cpp
# ${PROJECT_SOURCE_DIR}/src/BundleAdjustment.cpp
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/RGBDSlam/Feature.h
${PROJECT_SOURCE_DIR}/include/RGBDSlam/Track.h
# ${PROJECT_SOURCE_DIR}/include/RGBDSlam/Odometry.h
# ${PROJECT_SOURCE_DIR}/include/RGBDSlam/ICP.h
# ${PROJECT_SOURCE_DIR}/include/RGBDSlam/Motion.h
# ${PROJECT_SOURCE_DIR}/include/Room/RGBDSlam/BundleAdjustment.h
)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(g2o REQUIRED)

set(G2O_LIBRARIES
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
                      components)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib COMPONENT install-app
        LIBRARY DESTINATION lib COMPONENT install-app
        RUNTIME DESTINATION bin COMPONENT install-app)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/RGBDSlam
        DESTINATION include/RGBDSlam
        FILES_MATCHING PATTERN "*.h")

export(TARGETS ${PROJECT_NAME} NAMESPACE rgbd_slam::
      APPEND FILE ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake)

file(APPEND
    ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
"set(RGBDSLAM_LIBRARIES \${RGBDSLAM_LIBRARIES} ${PROJECT_NAME})\n")
