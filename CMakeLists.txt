cmake_minimum_required(VERSION 3.29)
project(Final)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tạo executable cho main.cpp
add_executable(Final main.cpp)

# Tạo executable cho test.cpp
add_executable(TestApp ncurses/test.cpp)

# Liên kết thư viện cho Windows
if (WIN32)
    target_link_libraries(Final PRIVATE user32 gdi32)
    target_link_libraries(TestApp PRIVATE user32 gdi32)
endif()

# Thêm thư mục include
target_include_directories(Final PRIVATE ${CMAKE_SOURCE_DIR})
target_include_directories(TestApp PRIVATE ${CMAKE_SOURCE_DIR})
