# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/vuwij/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/192.7142.39/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/vuwij/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/192.7142.39/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vuwij/catkin_ws/src/acrobot/src/acrobot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/uno_twi.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/uno_twi.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/uno_twi.dir/flags.make

CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp.obj: CMakeFiles/uno_twi.dir/flags.make
CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp.obj: /opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp.obj"
	/opt/local/arduino/hardware/tools/avr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp.obj -c /opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp

CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp.i"
	/opt/local/arduino/hardware/tools/avr/bin/avr-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp > CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp.i

CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp.s"
	/opt/local/arduino/hardware/tools/avr/bin/avr-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp -o CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp.s

CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c.obj: CMakeFiles/uno_twi.dir/flags.make
CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c.obj: /opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c.obj"
	/opt/local/arduino/hardware/tools/avr/bin/avr-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c.obj   -c /opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c

CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c.i"
	/opt/local/arduino/hardware/tools/avr/bin/avr-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c > CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c.i

CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c.s"
	/opt/local/arduino/hardware/tools/avr/bin/avr-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c -o CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c.s

# Object files for target uno_twi
uno_twi_OBJECTS = \
"CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp.obj" \
"CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c.obj"

# External object files for target uno_twi
uno_twi_EXTERNAL_OBJECTS =

libuno_twi.a: CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/Wire.cpp.obj
libuno_twi.a: CMakeFiles/uno_twi.dir/opt/local/arduino/hardware/arduino/avr/libraries/Wire/src/utility/twi.c.obj
libuno_twi.a: CMakeFiles/uno_twi.dir/build.make
libuno_twi.a: CMakeFiles/uno_twi.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libuno_twi.a"
	$(CMAKE_COMMAND) -P CMakeFiles/uno_twi.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/uno_twi.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/uno_twi.dir/build: libuno_twi.a

.PHONY : CMakeFiles/uno_twi.dir/build

CMakeFiles/uno_twi.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/uno_twi.dir/cmake_clean.cmake
.PHONY : CMakeFiles/uno_twi.dir/clean

CMakeFiles/uno_twi.dir/depend:
	cd /home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vuwij/catkin_ws/src/acrobot/src/acrobot /home/vuwij/catkin_ws/src/acrobot/src/acrobot /home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug /home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug /home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug/CMakeFiles/uno_twi.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/uno_twi.dir/depend
