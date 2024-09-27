find_package(Eigen3 CONFIG REQUIRED)

add_executable(assignment3 main.cpp)

target_link_libraries(assignment3 PRIVATE Eigen3::Eigen)
target_link_libraries(assignment3 PRIVATE math)