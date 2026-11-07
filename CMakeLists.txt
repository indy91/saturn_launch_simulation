# Version
cmake_minimum_required(VERSION 3.23)
#Project
project(saturn_launch_simulation)
#Executable
add_executable(saturn_launch_simulation WIN32)
# Sub-projects
add_subdirectory(src)
# wxWidgets
find_package(wxWidgets REQUIRED gl core base OPTIONAL_COMPONENTS net)
include(${wxWidgets_USE_FILE})
#libraries
target_link_libraries(saturn_launch_simulation ${wxWidgets_LIBRARIES})
#Executable output directory
set_target_properties(saturn_launch_simulation PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")