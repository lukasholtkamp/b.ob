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
CMAKE_SOURCE_DIR = /home/jkan67/jkan67/b.ob/src/motor_driver_library

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jkan67/jkan67/b.ob/src/motor_driver_library/build

# Include any dependencies generated for this target.
include CMakeFiles/motor_driver_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/motor_driver_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/motor_driver_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/motor_driver_test.dir/flags.make

CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.o: CMakeFiles/motor_driver_test.dir/flags.make
CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.o: ../test/motor_driver_test.cpp
CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.o: CMakeFiles/motor_driver_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jkan67/jkan67/b.ob/src/motor_driver_library/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.o -MF CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.o.d -o CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.o -c /home/jkan67/jkan67/b.ob/src/motor_driver_library/test/motor_driver_test.cpp

CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jkan67/jkan67/b.ob/src/motor_driver_library/test/motor_driver_test.cpp > CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.i

CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jkan67/jkan67/b.ob/src/motor_driver_library/test/motor_driver_test.cpp -o CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.s

# Object files for target motor_driver_test
motor_driver_test_OBJECTS = \
"CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.o"

# External object files for target motor_driver_test
motor_driver_test_EXTERNAL_OBJECTS =

motor_driver_test: CMakeFiles/motor_driver_test.dir/test/motor_driver_test.cpp.o
motor_driver_test: CMakeFiles/motor_driver_test.dir/build.make
motor_driver_test: /usr/lib/x86_64-linux-gnu/libfmt.so.8.1.1
motor_driver_test: CMakeFiles/motor_driver_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jkan67/jkan67/b.ob/src/motor_driver_library/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable motor_driver_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/motor_driver_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/motor_driver_test.dir/build: motor_driver_test
.PHONY : CMakeFiles/motor_driver_test.dir/build

CMakeFiles/motor_driver_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/motor_driver_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/motor_driver_test.dir/clean

CMakeFiles/motor_driver_test.dir/depend:
	cd /home/jkan67/jkan67/b.ob/src/motor_driver_library/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jkan67/jkan67/b.ob/src/motor_driver_library /home/jkan67/jkan67/b.ob/src/motor_driver_library /home/jkan67/jkan67/b.ob/src/motor_driver_library/build /home/jkan67/jkan67/b.ob/src/motor_driver_library/build /home/jkan67/jkan67/b.ob/src/motor_driver_library/build/CMakeFiles/motor_driver_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/motor_driver_test.dir/depend

