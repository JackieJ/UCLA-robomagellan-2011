## Project Setup
cmake_minimum_required(VERSION 2.4.6)
set(CMAKE_VERBOSE_MAKEFILE true)
project(AX3500)

## Configurations

# Allow for building shared libs override
IF(NOT BUILD_SHARED_LIBS)
	set(BUILD_SHARED_LIBS OFF)
ENDIF(NOT BUILD_SHARED_LIBS)

# Allow for build type override
IF(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
ENDIF(NOT CMAKE_BUILD_TYPE)

# set the default path for built libraries to the "lib" directory
IF(NOT LIBRARY_OUTPUT_PATH)
	set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF(NOT LIBRARY_OUTPUT_PATH)

## Configure build system

# Add the include folder to the include path
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add default source files
set(AX3500_SRCS src/AX3500.cpp)
# Add default header files
set(AX3500_HEADERS include/AX3500.h)

## Find and add Boost
find_package(Boost COMPONENTS system thread REQUIRED)
link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
set(AX3500_LINK_LIBS ${Boost_SYSTEM_LIBRARY}
                     ${Boost_THREAD_LIBRARY})

## Build the library

# Compile the AX3500 library
add_library(ax3500 ${AX3500_SRCS})
target_link_libraries(ax3500 ${AX3500_LINK_LIBS})

# Configure make install
IF(NOT CMAKE_INSTALL_PREFIX)
	SET(CMAKE_INSTALL_PREFIX /usr/local)
ENDIF(NOT CMAKE_INSTALL_PREFIX)

## Install setup

# Install the library
IF(NOT AX3500_DONT_SETUP_INSTALL)
	INSTALL(
		TARGETS ax3500
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
	)

	INSTALL(FILES ${AX3500_HEADERS} DESTINATION include)

	IF(NOT CMAKE_FIND_INSTALL_PATH)
		set(CMAKE_FIND_INSTALL_PATH ${CMAKE_ROOT})
	ENDIF(NOT CMAKE_FIND_INSTALL_PATH)

	ADD_CUSTOM_TARGET(uninstall @echo uninstall package)

	ADD_CUSTOM_COMMAND(
		COMMENT "uninstall package"
		COMMENT xargs ARGS rm < install_manifest.txt
		TARGET  uninstall
	)
ENDIF(NOT AX3500_DONT_SETUP_INSTALL)
