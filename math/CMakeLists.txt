find_package(Eigen3 CONFIG REQUIRED)

add_library(math STATIC
        math.cpp
        include/math/math.h)

target_include_directories(math PUBLIC include/math/math.h)