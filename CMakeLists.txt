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
        src/data_structures/rtrees/common/bounding_box.hpp
        src/data_structures/rtrees/common/edge_ptr.hpp
        src/data_structures/rtrees/common/reinsert_entries.hpp
        src/data_structures/rtrees/rtree_config.hpp
        src/data_structures/rtrees/core/rtree_node.hpp
        src/data_structures/rtrees/core/rtree_node.cpp
        src/data_structures/rtrees/common/search_state.hpp
        src/data_structures/rtrees/core/rtree_base.hpp
        src/data_structures/rtrees/core/rtree_base.cpp
        src/data_structures/rtrees/common/tree_path.hpp
        src/data_structures/rtrees/common/parent_candidates.hpp
        src/data_structures/rtrees/rtree/rtree.hpp
        src/data_structures/rtrees/rtree/rtree.cpp
)

target_include_directories(MapPilot PRIVATE src)
target_compile_options(MapPilot PRIVATE -Wall -Wextra -O2)