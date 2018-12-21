cmake_minimum_required(VERSION 3.5)

project(JSONMinimalExample)

find_package(nlohmann_json REQUIRED)

set(PROJECT_SRCS ${PROJECT_SOURCE_DIR}/src/JSONMinimalExample.cpp)

source_group("src" FILES ${PROJECT_SRCS})

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} PUBLIC nlohmann_json)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin COMPONENT install-app)
