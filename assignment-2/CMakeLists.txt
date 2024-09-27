find_package(Eigen3 CONFIG REQUIRED)

add_executable(assignment2 main.cpp
        ../assignment-1/main.cpp
        ../math/include/math/math.h)

target_link_libraries(assignment2 PRIVATE Eigen3::Eigen)
