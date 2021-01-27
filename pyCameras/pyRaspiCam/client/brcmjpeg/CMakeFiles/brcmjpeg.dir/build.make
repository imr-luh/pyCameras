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
include client/brcmjpeg/CMakeFiles/brcmjpeg.dir/depend.make

# Include the progress variables for this target.
include client/brcmjpeg/CMakeFiles/brcmjpeg.dir/progress.make

# Include the compile flags for this target's objects.
include client/brcmjpeg/CMakeFiles/brcmjpeg.dir/flags.make

client/brcmjpeg/CMakeFiles/brcmjpeg.dir/brcmjpeg.o: client/brcmjpeg/CMakeFiles/brcmjpeg.dir/flags.make
client/brcmjpeg/CMakeFiles/brcmjpeg.dir/brcmjpeg.o: interface/mmal/client/brcmjpeg/brcmjpeg.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object client/brcmjpeg/CMakeFiles/brcmjpeg.dir/brcmjpeg.o"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/client/brcmjpeg && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/brcmjpeg.dir/brcmjpeg.o   -c /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/client/brcmjpeg/brcmjpeg.c

client/brcmjpeg/CMakeFiles/brcmjpeg.dir/brcmjpeg.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/brcmjpeg.dir/brcmjpeg.i"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/client/brcmjpeg && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/client/brcmjpeg/brcmjpeg.c > CMakeFiles/brcmjpeg.dir/brcmjpeg.i

client/brcmjpeg/CMakeFiles/brcmjpeg.dir/brcmjpeg.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/brcmjpeg.dir/brcmjpeg.s"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/client/brcmjpeg && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/client/brcmjpeg/brcmjpeg.c -o CMakeFiles/brcmjpeg.dir/brcmjpeg.s

# Object files for target brcmjpeg
brcmjpeg_OBJECTS = \
"CMakeFiles/brcmjpeg.dir/brcmjpeg.o"

# External object files for target brcmjpeg
brcmjpeg_EXTERNAL_OBJECTS =

client/brcmjpeg/libbrcmjpeg.so: client/brcmjpeg/CMakeFiles/brcmjpeg.dir/brcmjpeg.o
client/brcmjpeg/libbrcmjpeg.so: client/brcmjpeg/CMakeFiles/brcmjpeg.dir/build.make
client/brcmjpeg/libbrcmjpeg.so: core/libmmal_core.so
client/brcmjpeg/libbrcmjpeg.so: util/libmmal_util.so
client/brcmjpeg/libbrcmjpeg.so: vc/libmmal_vc_client.so
client/brcmjpeg/libbrcmjpeg.so: client/brcmjpeg/CMakeFiles/brcmjpeg.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared library libbrcmjpeg.so"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/client/brcmjpeg && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/brcmjpeg.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
client/brcmjpeg/CMakeFiles/brcmjpeg.dir/build: client/brcmjpeg/libbrcmjpeg.so

.PHONY : client/brcmjpeg/CMakeFiles/brcmjpeg.dir/build

client/brcmjpeg/CMakeFiles/brcmjpeg.dir/clean:
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/client/brcmjpeg && $(CMAKE_COMMAND) -P CMakeFiles/brcmjpeg.dir/cmake_clean.cmake
.PHONY : client/brcmjpeg/CMakeFiles/brcmjpeg.dir/clean

client/brcmjpeg/CMakeFiles/brcmjpeg.dir/depend:
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/client/brcmjpeg /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/client/brcmjpeg /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/client/brcmjpeg/CMakeFiles/brcmjpeg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : client/brcmjpeg/CMakeFiles/brcmjpeg.dir/depend

