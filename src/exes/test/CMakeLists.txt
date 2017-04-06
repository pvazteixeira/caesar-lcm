file(GLOB_RECURSE implem_files *.cpp)

set(EXE_NAME ${POD_NAME}-test)

add_executable(${EXE_NAME} ${implem_files})

#add_definitions(-std=c++0x -lpthread -pg -fopenmp -Wall -Wfatal-errors -Wextra -g ) # use this for debugging

# pkg-config packages
#find_package(Boost COMPONENTS system REQUIRED)

set(REQUIRED_PACKAGES eigen3 )

pods_use_pkg_config_packages(${EXE_NAME} ${REQUIRED_PACKAGES})

# target_link_libraries(${EXE_NAME} ${Boost_SYSTEM_LIBRARY})

pods_install_executables(${EXE_NAME})
