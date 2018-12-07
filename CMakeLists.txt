cmake_minimum_required(VERSION 3.11)

project(realsense-rgbd-slam LANGUAGES CXX)

set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(COMPILE_FLAGS "-W -Wall -Werror -fsigned-char -freg-struct-return
  -Wextra -Wpointer-arith -Wcast-qual -Wshadow -Wpedantic")
  add_compile_options(${COMPILE_FLAGS})
endif()

if(NOT TARGET uninstall)
  configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
      IMMEDIATE @ONLY)

  add_custom_target(uninstall
      COMMAND ${CMAKE_COMMAND} -P
      ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()

if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
  export(TARGETS FILE ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake)
endif()

file(WRITE ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
    "include(CMakeFindDependencyMacro)\n"
    "include(\${CMAKE_CURRENT_LIST_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake)\n")

add_subdirectory(modules)
add_subdirectory(apps)
add_subdirectory(doc)

install(FILES ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
        DESTINATION lib/cmake/${CMAKE_PROJECT_NAME})

install(EXPORT ${CMAKE_PROJECT_NAME}Targets NAMESPACE room::
        DESTINATION lib/cmake/${CMAKE_PROJECT_NAME})

if(NOT TARGET install-app)
  add_custom_target(install-app COMMAND
                    "${CMAKE_COMMAND}" -DCMAKE_INSTALL_COMPONENT=install-app
                    -P "${CMAKE_BINARY_DIR}/cmake_install.cmake")
endif()
