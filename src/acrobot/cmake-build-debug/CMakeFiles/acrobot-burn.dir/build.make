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

# Utility rule file for acrobot-burn.

# Include the progress variables for this target.
include CMakeFiles/acrobot-burn.dir/progress.make

CMakeFiles/acrobot-burn: acrobot.elf
	/opt/local/arduino/hardware/tools/avr/bin/avrdude -C/opt/local/arduino/hardware/tools/avr/etc/avrdude.conf -cstk500v2 -Pusb -patmega328p -V -Uflash:w:/home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug/acrobot.hex

acrobot-burn: CMakeFiles/acrobot-burn
acrobot-burn: CMakeFiles/acrobot-burn.dir/build.make

.PHONY : acrobot-burn

# Rule to build all files generated by this target.
CMakeFiles/acrobot-burn.dir/build: acrobot-burn

.PHONY : CMakeFiles/acrobot-burn.dir/build

CMakeFiles/acrobot-burn.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/acrobot-burn.dir/cmake_clean.cmake
.PHONY : CMakeFiles/acrobot-burn.dir/clean

CMakeFiles/acrobot-burn.dir/depend:
	cd /home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vuwij/catkin_ws/src/acrobot/src/acrobot /home/vuwij/catkin_ws/src/acrobot/src/acrobot /home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug /home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug /home/vuwij/catkin_ws/src/acrobot/src/acrobot/cmake-build-debug/CMakeFiles/acrobot-burn.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/acrobot-burn.dir/depend

