# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/abhishek/Documents/instrument-and-actuator-development/safety_controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/abhishek/Documents/instrument-and-actuator-development/safety_controller/build

# Include any dependencies generated for this target.
include CMakeFiles/safety_controller.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/safety_controller.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/safety_controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/safety_controller.dir/flags.make

CMakeFiles/safety_controller.dir/safety_controller.cpp.o: CMakeFiles/safety_controller.dir/flags.make
CMakeFiles/safety_controller.dir/safety_controller.cpp.o: ../safety_controller.cpp
CMakeFiles/safety_controller.dir/safety_controller.cpp.o: CMakeFiles/safety_controller.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abhishek/Documents/instrument-and-actuator-development/safety_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/safety_controller.dir/safety_controller.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/safety_controller.dir/safety_controller.cpp.o -MF CMakeFiles/safety_controller.dir/safety_controller.cpp.o.d -o CMakeFiles/safety_controller.dir/safety_controller.cpp.o -c /home/abhishek/Documents/instrument-and-actuator-development/safety_controller/safety_controller.cpp

CMakeFiles/safety_controller.dir/safety_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/safety_controller.dir/safety_controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abhishek/Documents/instrument-and-actuator-development/safety_controller/safety_controller.cpp > CMakeFiles/safety_controller.dir/safety_controller.cpp.i

CMakeFiles/safety_controller.dir/safety_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/safety_controller.dir/safety_controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abhishek/Documents/instrument-and-actuator-development/safety_controller/safety_controller.cpp -o CMakeFiles/safety_controller.dir/safety_controller.cpp.s

# Object files for target safety_controller
safety_controller_OBJECTS = \
"CMakeFiles/safety_controller.dir/safety_controller.cpp.o"

# External object files for target safety_controller
safety_controller_EXTERNAL_OBJECTS =

safety_controller: CMakeFiles/safety_controller.dir/safety_controller.cpp.o
safety_controller: CMakeFiles/safety_controller.dir/build.make
safety_controller: CMakeFiles/safety_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/abhishek/Documents/instrument-and-actuator-development/safety_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable safety_controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/safety_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/safety_controller.dir/build: safety_controller
.PHONY : CMakeFiles/safety_controller.dir/build

CMakeFiles/safety_controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/safety_controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/safety_controller.dir/clean

CMakeFiles/safety_controller.dir/depend:
	cd /home/abhishek/Documents/instrument-and-actuator-development/safety_controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/abhishek/Documents/instrument-and-actuator-development/safety_controller /home/abhishek/Documents/instrument-and-actuator-development/safety_controller /home/abhishek/Documents/instrument-and-actuator-development/safety_controller/build /home/abhishek/Documents/instrument-and-actuator-development/safety_controller/build /home/abhishek/Documents/instrument-and-actuator-development/safety_controller/build/CMakeFiles/safety_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/safety_controller.dir/depend

