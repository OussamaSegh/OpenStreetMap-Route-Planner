# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/osghaier/OpenStreetMap-Route-Planner

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/osghaier/OpenStreetMap-Route-Planner/build

# Utility rule file for Continuous.

# Include the progress variables for this target.
include thirdparty/pugixml/CMakeFiles/Continuous.dir/progress.make

thirdparty/pugixml/CMakeFiles/Continuous:
	cd /home/osghaier/OpenStreetMap-Route-Planner/build/thirdparty/pugixml && /usr/bin/ctest -D Continuous

Continuous: thirdparty/pugixml/CMakeFiles/Continuous
Continuous: thirdparty/pugixml/CMakeFiles/Continuous.dir/build.make

.PHONY : Continuous

# Rule to build all files generated by this target.
thirdparty/pugixml/CMakeFiles/Continuous.dir/build: Continuous

.PHONY : thirdparty/pugixml/CMakeFiles/Continuous.dir/build

thirdparty/pugixml/CMakeFiles/Continuous.dir/clean:
	cd /home/osghaier/OpenStreetMap-Route-Planner/build/thirdparty/pugixml && $(CMAKE_COMMAND) -P CMakeFiles/Continuous.dir/cmake_clean.cmake
.PHONY : thirdparty/pugixml/CMakeFiles/Continuous.dir/clean

thirdparty/pugixml/CMakeFiles/Continuous.dir/depend:
	cd /home/osghaier/OpenStreetMap-Route-Planner/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/osghaier/OpenStreetMap-Route-Planner /home/osghaier/OpenStreetMap-Route-Planner/thirdparty/pugixml /home/osghaier/OpenStreetMap-Route-Planner/build /home/osghaier/OpenStreetMap-Route-Planner/build/thirdparty/pugixml /home/osghaier/OpenStreetMap-Route-Planner/build/thirdparty/pugixml/CMakeFiles/Continuous.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : thirdparty/pugixml/CMakeFiles/Continuous.dir/depend

