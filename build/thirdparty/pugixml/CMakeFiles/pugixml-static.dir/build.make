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

# Include any dependencies generated for this target.
include thirdparty/pugixml/CMakeFiles/pugixml-static.dir/depend.make

# Include the progress variables for this target.
include thirdparty/pugixml/CMakeFiles/pugixml-static.dir/progress.make

# Include the compile flags for this target's objects.
include thirdparty/pugixml/CMakeFiles/pugixml-static.dir/flags.make

thirdparty/pugixml/CMakeFiles/pugixml-static.dir/src/pugixml.cpp.o: thirdparty/pugixml/CMakeFiles/pugixml-static.dir/flags.make
thirdparty/pugixml/CMakeFiles/pugixml-static.dir/src/pugixml.cpp.o: ../thirdparty/pugixml/src/pugixml.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/osghaier/OpenStreetMap-Route-Planner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object thirdparty/pugixml/CMakeFiles/pugixml-static.dir/src/pugixml.cpp.o"
	cd /home/osghaier/OpenStreetMap-Route-Planner/build/thirdparty/pugixml && /bin/g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pugixml-static.dir/src/pugixml.cpp.o -c /home/osghaier/OpenStreetMap-Route-Planner/thirdparty/pugixml/src/pugixml.cpp

thirdparty/pugixml/CMakeFiles/pugixml-static.dir/src/pugixml.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pugixml-static.dir/src/pugixml.cpp.i"
	cd /home/osghaier/OpenStreetMap-Route-Planner/build/thirdparty/pugixml && /bin/g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/osghaier/OpenStreetMap-Route-Planner/thirdparty/pugixml/src/pugixml.cpp > CMakeFiles/pugixml-static.dir/src/pugixml.cpp.i

thirdparty/pugixml/CMakeFiles/pugixml-static.dir/src/pugixml.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pugixml-static.dir/src/pugixml.cpp.s"
	cd /home/osghaier/OpenStreetMap-Route-Planner/build/thirdparty/pugixml && /bin/g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/osghaier/OpenStreetMap-Route-Planner/thirdparty/pugixml/src/pugixml.cpp -o CMakeFiles/pugixml-static.dir/src/pugixml.cpp.s

# Object files for target pugixml-static
pugixml__static_OBJECTS = \
"CMakeFiles/pugixml-static.dir/src/pugixml.cpp.o"

# External object files for target pugixml-static
pugixml__static_EXTERNAL_OBJECTS =

../lib/libpugixml.a: thirdparty/pugixml/CMakeFiles/pugixml-static.dir/src/pugixml.cpp.o
../lib/libpugixml.a: thirdparty/pugixml/CMakeFiles/pugixml-static.dir/build.make
../lib/libpugixml.a: thirdparty/pugixml/CMakeFiles/pugixml-static.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/osghaier/OpenStreetMap-Route-Planner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../../lib/libpugixml.a"
	cd /home/osghaier/OpenStreetMap-Route-Planner/build/thirdparty/pugixml && $(CMAKE_COMMAND) -P CMakeFiles/pugixml-static.dir/cmake_clean_target.cmake
	cd /home/osghaier/OpenStreetMap-Route-Planner/build/thirdparty/pugixml && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pugixml-static.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
thirdparty/pugixml/CMakeFiles/pugixml-static.dir/build: ../lib/libpugixml.a

.PHONY : thirdparty/pugixml/CMakeFiles/pugixml-static.dir/build

thirdparty/pugixml/CMakeFiles/pugixml-static.dir/clean:
	cd /home/osghaier/OpenStreetMap-Route-Planner/build/thirdparty/pugixml && $(CMAKE_COMMAND) -P CMakeFiles/pugixml-static.dir/cmake_clean.cmake
.PHONY : thirdparty/pugixml/CMakeFiles/pugixml-static.dir/clean

thirdparty/pugixml/CMakeFiles/pugixml-static.dir/depend:
	cd /home/osghaier/OpenStreetMap-Route-Planner/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/osghaier/OpenStreetMap-Route-Planner /home/osghaier/OpenStreetMap-Route-Planner/thirdparty/pugixml /home/osghaier/OpenStreetMap-Route-Planner/build /home/osghaier/OpenStreetMap-Route-Planner/build/thirdparty/pugixml /home/osghaier/OpenStreetMap-Route-Planner/build/thirdparty/pugixml/CMakeFiles/pugixml-static.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : thirdparty/pugixml/CMakeFiles/pugixml-static.dir/depend

