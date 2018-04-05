file(GLOB_RECURSE implem_files *.cpp)

set(EXE_NAME ${POD_NAME}-test)

add_executable(${EXE_NAME} ${implem_files})

#add_definitions(-std=c++0x -lpthread -pg -fopenmp -Wall -Wfatal-errors -Wextra -g ) # use this for debugging

add_definitions(-std=c++11)

# 1. cmake-supported libraries
find_package(PkgConfig REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common)
message("PCL_FOUND=${PCL_FOUND}")
message("PCL_INCLUDE_DIRS=${PCL_INCLUDE_DIRS}")
message("PCL_LIBRARY_DIRS=${PCL_LIBRARY_DIRS}")
message("PCL_DEFINITIONS=${PCL_DEFINITIONS}")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# 3. pkg-config packages
find_package(Boost COMPONENTS system REQUIRED)

set(REQUIRED_PACKAGES eigen3 lcm caesar )

pods_use_pkg_config_packages(${EXE_NAME} ${REQUIRED_PACKAGES})

target_link_libraries(${EXE_NAME} ${Boost_SYSTEM_LIBRARY} ${PCL_COMMON_LIBRARIES})

pods_install_executables(${EXE_NAME})
