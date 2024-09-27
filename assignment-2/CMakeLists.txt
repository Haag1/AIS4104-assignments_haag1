find_package(Eigen3 CONFIG REQUIRED)

add_executable(assignment2 main.cpp)

target_link_libraries(assignment2 PRIVATE Eigen3::Eigen)
target_link_libraries(assignment2 PRIVATE math)
