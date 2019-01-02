cmake_minimum_required(VERSION 3.5)
project(room_rgbdslam)


set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Feature.cpp
${PROJECT_SOURCE_DIR}/src/Odometry.cpp
${PROJECT_SOURCE_DIR}/src/ICP.cpp
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Room/RGBDSlam/Feature.h
${PROJECT_SOURCE_DIR}/include/Room/RGBDSlam/Odometry.h
${PROJECT_SOURCE_DIR}/include/Room/RGBDSlam/ICP.h
)

source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})

target_include_directories(${PROJECT_NAME} PUBLIC
                          $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>)

target_link_libraries(${PROJECT_NAME}
                      ${OpenCV_LIBS}
                      spdlog::spdlog
                      ceres)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib COMPONENT install-app
        LIBRARY DESTINATION lib COMPONENT install-app
        RUNTIME DESTINATION bin COMPONENT install-app)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/Room/RGBDSlam
        DESTINATION include/Room/RGBDSlam
        FILES_MATCHING PATTERN "*.h")

export(TARGETS ${PROJECT_NAME}
      APPEND FILE ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake)

file(APPEND
    ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
"set(ROOM_RGBDSLAM_LIBRARIES \${ROOM_RGBDSLAM_LIBRARIES} ${PROJECT_NAME})\n")
