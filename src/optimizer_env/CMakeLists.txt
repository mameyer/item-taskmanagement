project(follower_environment)
set(PROJECT_VERSION 1.0)
set(PROJECT_DESCRIPTION "follower optimization environment")

find_package(PkgConfig REQUIRED)
find_package(lib_manager)
lib_defaults()
define_module_info()

add_definitions(
	-std=c++11
)

if(WIN32)
  # this fixes the error 998 from the LibManager
  SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--enable-auto-import -Wall")
  SET(CMAKE_MODULE_LINKER_FLAGS "-Wl,--enable-auto-import -Wall")
else(WIN32)
  SET(CMAKE_CXX_FLAGS "-fPIC -Wall")
endif(WIN32)

cmake_minimum_required(VERSION 2.6)

pkg_check_modules(PKGCONFIG lib_manager
                            configmaps
                            bolero
                            base-types
			    base-lib
			    eigen3)
                 
include_directories(${PKGCONFIG_INCLUDE_DIRS})
link_directories(${PKGCONFIG_LIBRARY_DIRS})

set(SOURCES FollowerOptimizerEnvironment.cpp)
set(HEADERS FollowerOptimizerEnvironment.hpp)
set(PKGCONF follower_environment.pc)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${PKGCONFIG_LIBRARIES})

if(WIN32)
  set(LIB_INSTALL_DIR bin) # .dll are in PATH, like executables
else(WIN32)
  set(LIB_INSTALL_DIR lib)
endif(WIN32)

set(_INSTALL_DESTINATIONS
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION ${LIB_INSTALL_DIR}
  ARCHIVE DESTINATION lib
)

# Install the library into the lib folder
install(TARGETS ${PROJECT_NAME} ${_INSTALL_DESTINATIONS})

# Install pkg-config
install(FILES ${PKGCONF} DESTINATION lib/pkgconfig)

# Install headers
install(FILES ${HEADERS} DESTINATION include/bolero/${PROJECT_NAME})
# add_subdirectory(test)