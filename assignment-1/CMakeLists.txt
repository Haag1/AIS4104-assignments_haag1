find_package(Eigen3 CONFIG REQUIRED)

add_executable(assignment1 main.cpp)

target_link_libraries(assignment1 PRIVATE Eigen3::Eigen)
target_link_libraries(assignment1 PRIVATE math)