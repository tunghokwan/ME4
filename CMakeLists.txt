cmake_minimum_required(VERSION 3.10.0)
project(mysocket CXX)


add_subdirectory(linux_demo)
add_subdirectory( ${SOURCE_DIR} /home/xiaoqiang/Desktop/dllm/lib/json [EXCLUDE_FROM_ALL] )

include_directories(/home/xiaoqiang/Desktop/dllm/zeroerr/exo_control_cpp-master/include)
include_directories(/home/xiaoqiang/Desktop/dllm/zeroerr/linux_demo)

#add_subdirectory( ${SOURCE_DIR} /home/xiaoqiang/Desktop/lib/websocketpp/websocketpp [EXCLUDE_FROM_ALL] )
add_library(websocketpp SHARED IMPORTED)

set_target_properties(websocketpp PROPERTIES IMPORTED_LOCATION /home/xiaoqiang/Desktop/dllm/zeroerr/exo_control_cpp-master/include/websocketpp/client.hpp)


add_executable(mysocket  mysocket.cpp )
#add_library(mysocket  mysocket.cpp )


target_link_libraries(mysocket PUBLIC ZeroErr_control)

target_include_directories(mysocket PUBLIC
"${PROJECT_BINARY_DIR}"
"${PROJECT_SOURCE_DIR}/linux_demo"
)

#target_link_libraries(mysocket PRIVATE nlohmann_json::nlohmann_json)


#target_link_libraries(mysocket PRIVATE /home/xiaoqiang/Desktop/lib/websocketpp/websocketpp/client.hpp)

target_link_libraries(mysocket PRIVATE websocketpp)



set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS -pthread)
set(CMAKE_CXX_STANDARD_REQUIRED True)


#home/roborn/Desktop/dllm/