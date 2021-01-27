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
CMAKE_SOURCE_DIR = /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam

# Include any dependencies generated for this target.
include CMakeFiles/mmal.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mmal.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mmal.dir/flags.make

CMakeFiles/mmal.dir/util/mmal_util.o: CMakeFiles/mmal.dir/flags.make
CMakeFiles/mmal.dir/util/mmal_util.o: interface/mmal/util/mmal_util.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/mmal.dir/util/mmal_util.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mmal.dir/util/mmal_util.o   -c /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_util.c

CMakeFiles/mmal.dir/util/mmal_util.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal.dir/util/mmal_util.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_util.c > CMakeFiles/mmal.dir/util/mmal_util.i

CMakeFiles/mmal.dir/util/mmal_util.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal.dir/util/mmal_util.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_util.c -o CMakeFiles/mmal.dir/util/mmal_util.s

# Object files for target mmal
mmal_OBJECTS = \
"CMakeFiles/mmal.dir/util/mmal_util.o"

# External object files for target mmal
mmal_EXTERNAL_OBJECTS =

libmmal.so: CMakeFiles/mmal.dir/util/mmal_util.o
libmmal.so: CMakeFiles/mmal.dir/build.make
libmmal.so: vc/libmmal_vc_client.so
libmmal.so: components/libmmal_components.so
libmmal.so: core/libmmal_core.so
libmmal.so: util/libmmal_util.so
libmmal.so: CMakeFiles/mmal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared library libmmal.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mmal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mmal.dir/build: libmmal.so

.PHONY : CMakeFiles/mmal.dir/build

CMakeFiles/mmal.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mmal.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mmal.dir/clean

CMakeFiles/mmal.dir/depend:
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/CMakeFiles/mmal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mmal.dir/depend

