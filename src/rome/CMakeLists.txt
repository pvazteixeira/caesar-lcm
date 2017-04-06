# grab relevant files
file(GLOB implementation_files *.c *.cpp)
file(GLOB header_files *.h *.hpp)

set(LIBRARY_NAME rome)

add_library(${LIBRARY_NAME} SHARED ${implementation_files})

# 1. cmake-supported libraries
# find_package(PkgConfig REQUIRED)
# find_package(PCL REQUIRED )
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# 2. non-pkg-config libraries
# set(REQUIRED_LIBS $F

# 3. pkg-config libraries
set(REQUIRED_PACKAGES eigen3)
pods_use_pkg_config_packages(${LIBRARY_NAME} ${REQUIRED_PACKAGES})

# install library
pods_install_libraries(${LIBRARY_NAME})
pods_install_headers(${header_files} DESTINATION ${LIBRARY_NAME})
pods_install_pkg_config_file(${LIBRARY_NAME}
  LIBS -l${LIBRARY_NAME}
  VERSION 0.0.1)
