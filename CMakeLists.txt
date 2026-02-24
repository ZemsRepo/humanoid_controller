cmake_minimum_required(VERSION 3.8)
project(humanoid_controller)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

ament_auto_add_library(
    ${PROJECT_NAME}
    SHARED
    DIRECTORY src
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface humanoid_controller.xml)

target_include_directories(
    ${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} 
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
)

ament_auto_package(
    INSTALL_TO_SHARE config launch
)

# Author Linter Custom Targets
find_program(PYTHON3_EXECUTABLE python3)
if(PYTHON3_EXECUTABLE)
  set(AUTHOR_LINTER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/utils/marks/author_linter.py)
  set(AUTHOR_LINTER_CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/utils/marks/author_config.yaml)
  set(AUTHOR_LINTER_AUTHOR "Fulong Yin" CACHE STRING "Author name for author linter")
  set(AUTHOR_LINTER_ORG "IO-AI.tech" CACHE STRING "Organization name for author linter")
  
  # Check if config file exists
  if(EXISTS ${AUTHOR_LINTER_CONFIG})
    set(AUTHOR_LINTER_ARGS --config ${AUTHOR_LINTER_CONFIG})
  else()
    set(AUTHOR_LINTER_ARGS --author "${AUTHOR_LINTER_AUTHOR}" --org "${AUTHOR_LINTER_ORG}")
  endif()
  
  # Check target
  add_custom_target(check_authors
    COMMAND ${PYTHON3_EXECUTABLE} ${AUTHOR_LINTER_SCRIPT} --check ${AUTHOR_LINTER_ARGS} src/ include/
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Checking author information in C++ files"
    VERBATIM
  )
  
  # Fix target
  add_custom_target(fix_authors
    COMMAND ${PYTHON3_EXECUTABLE} ${AUTHOR_LINTER_SCRIPT} --fix ${AUTHOR_LINTER_ARGS} src/ include/
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Fixing author information in C++ files"
    VERBATIM
  )
  
  message(STATUS "Author linter targets available: check_authors, fix_authors")
endif()
