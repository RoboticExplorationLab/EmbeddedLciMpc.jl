project(EmbeddedLciMpc)

cmake_minimum_required(VERSION 2.8.12)

# Find julia library 
execute_process(
    COMMAND julia --startup-file=no -e "print(dirname(Sys.BINDIR))"
    OUTPUT_VARIABLE Julia_BINARY_DIRECTORY
)

include_directories(${Julia_BINARY_DIRECTORY}/include/julia)
link_directories(${Julia_BINARY_DIRECTORY}/lib)

add_executable(main src/main.cpp)
target_link_libraries(main julia)