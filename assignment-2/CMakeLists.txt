find_package(Eigen3 CONFIG REQUIRED)

add_executable(session_1 main.cpp)

target_link_libraries(session_1 PRIVATE Eigen3::Eigen)
