cmake_minimum_required(VERSION 3.5)

project(SlamPipeline)

set(PROJECT_SRCS ${PROJECT_SOURCE_DIR}/src/SlamPipeline.cpp)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} PUBLIC rgbdslam)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin COMPONENT install-app)
