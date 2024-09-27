find_package(Eigen3 CONFIG REQUIRED)

add_executable(assignment3 main.cpp
        ../assignment-3/main.cpp
        ../math/include/math/math.h)

target_link_libraries(assignment3 PRIVATE Eigen3::Eigen)
