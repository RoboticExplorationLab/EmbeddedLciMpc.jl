project(EmbeddedLciMpc)

cmake_minimum_required(VERSION 2.8.12)

# Find Eigen3 
find_package(Eigen3)
include_directories( ${EIGEN3_INCLUDE_DIR})

# Find julia library 
execute_process(
    COMMAND julia --startup-file=no -e "print(dirname(Sys.BINDIR))"
    OUTPUT_VARIABLE Julia_BINARY_DIRECTORY
)

include_directories(${Julia_BINARY_DIRECTORY}/include/julia)
link_directories(${Julia_BINARY_DIRECTORY}/lib)

include_directories(src/include)

add_library(juliaCpp_lib 
    src/juliaCpp.cpp
)
target_link_libraries(juliaCpp_lib julia)

# Expose the public includes
set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src/include
                                 ${Julia_BINARY_DIRECTORY}/include/julia
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

set(${PROJECT_NAME}_LIB_DIRS ${Julia_BINARY_DIRECTORY}/lib
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
message(STATUS ${Julia_BINARY_DIRECTORY}/include/julia)
add_executable(main src/main.cpp)
target_link_libraries(main julia juliaCpp_lib)

# add_executable(testCustomVector src/tests/testCustomVector.cpp)
# target_link_libraries(testCustomVector julia juliaCpp_lib)

# add_executable(testJuliaCall src/tests/testJuliaCall.cpp)
# target_link_libraries(testJuliaCall julia juliaCpp_lib)
