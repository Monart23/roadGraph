cmake_minimum_required(VERSION 3.10)
project(RoadGraph)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Create shared library
add_library(roadgraph SHARED road_graph_library.cpp)

target_compile_options(roadgraph PRIVATE -Wall -Wextra -Wpedantic)

# Create executable that links against the shared library
add_executable(road_graph_utility road_graph_utility.cpp)

target_link_libraries(road_graph_utility PRIVATE roadgraph)

# Installation rules
install(TARGETS roadgraph LIBRARY DESTINATION lib)
install(FILES road_graph_library.h DESTINATION include)
install(TARGETS road_graph_utility RUNTIME DESTINATION bin)

# Uninstallation target
add_custom_target(remove
    COMMAND rm -rf ${CMAKE_INSTALL_PREFIX}/lib/libroadgraph.so
    COMMAND rm -rf ${CMAKE_INSTALL_PREFIX}/include/road_graph_library.h
    COMMAND rm -rf ${CMAKE_INSTALL_PREFIX}/bin/road_graph_utility
    COMMENT "Removing installed files")
