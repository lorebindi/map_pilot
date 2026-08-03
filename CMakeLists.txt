cmake_minimum_required(VERSION 3.20)
project(MapPilot CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(MapPilot
        src/main.cpp
        src/graph/graph.cpp
        src/graph/heuristics.cpp
        src/graph/highways.cpp
        src/utils/utm_converter.cpp
        src/data_structures/min_heap/min_heap.cpp
        src/data_structures/rtrees/bounding_box.hpp
        src/data_structures/rtrees/edge_ptr.hpp
        src/data_structures/rtrees/reinsert_entries.hpp
        src/data_structures/rtrees/rtree_config.hpp
        src/data_structures/rtrees/rtree_node.hpp
        src/data_structures/rtrees/search_state.hpp
        src/data_structures/rtrees/rtree_base.hpp
        src/data_structures/rtrees/rtree_base.cpp
        src/data_structures/rtrees/tree_path.hpp
)

target_include_directories(MapPilot PRIVATE src)
target_compile_options(MapPilot PRIVATE -Wall -Wextra -O2)