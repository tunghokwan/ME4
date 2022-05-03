cmake_minimum_required(VERSION 3.10.0)
project(ZeroErr_control)


add_library( libECanVci SHARED IMPORTED )
# You can define two import-locations: one for debug and one for release.
set_target_properties( libECanVci PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libECanVci.so.1)


add_library( libusb-1.0 SHARED IMPORTED )
set_target_properties(  libusb-1.0 PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libusb-1.0.so)



add_library( libusb SHARED IMPORTED )
set_target_properties( libusb PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libusb.so)

 
#add lib or add exe
#add_executable(ZeroErr_control ZeroErr_control.cpp)
add_library(ZeroErr_control ZeroErr_control.cpp)



target_link_libraries(${PROJECT_NAME} libECanVci)
target_link_libraries(${PROJECT_NAME} libusb-1.0)
target_link_libraries(${PROJECT_NAME} libusb)

target_include_directories(ZeroErr_control
          INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
          )


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
