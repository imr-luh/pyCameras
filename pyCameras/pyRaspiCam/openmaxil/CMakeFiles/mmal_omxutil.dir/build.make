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
include openmaxil/CMakeFiles/mmal_omxutil.dir/depend.make

# Include the progress variables for this target.
include openmaxil/CMakeFiles/mmal_omxutil.dir/progress.make

# Include the compile flags for this target's objects.
include openmaxil/CMakeFiles/mmal_omxutil.dir/flags.make

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params.o: openmaxil/CMakeFiles/mmal_omxutil.dir/flags.make
openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params.o: interface/mmal/openmaxil/mmalomx_util_params.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params.o"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mmal_omxutil.dir/mmalomx_util_params.o   -c /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params.c

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_omxutil.dir/mmalomx_util_params.i"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params.c > CMakeFiles/mmal_omxutil.dir/mmalomx_util_params.i

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_omxutil.dir/mmalomx_util_params.s"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params.c -o CMakeFiles/mmal_omxutil.dir/mmalomx_util_params.s

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_audio.o: openmaxil/CMakeFiles/mmal_omxutil.dir/flags.make
openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_audio.o: interface/mmal/openmaxil/mmalomx_util_params_audio.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_audio.o"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_audio.o   -c /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params_audio.c

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_audio.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_audio.i"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params_audio.c > CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_audio.i

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_audio.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_audio.s"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params_audio.c -o CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_audio.s

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_video.o: openmaxil/CMakeFiles/mmal_omxutil.dir/flags.make
openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_video.o: interface/mmal/openmaxil/mmalomx_util_params_video.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_video.o"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_video.o   -c /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params_video.c

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_video.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_video.i"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params_video.c > CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_video.i

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_video.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_video.s"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params_video.c -o CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_video.s

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_camera.o: openmaxil/CMakeFiles/mmal_omxutil.dir/flags.make
openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_camera.o: interface/mmal/openmaxil/mmalomx_util_params_camera.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_camera.o"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_camera.o   -c /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params_camera.c

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_camera.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_camera.i"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params_camera.c > CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_camera.i

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_camera.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_camera.s"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params_camera.c -o CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_camera.s

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_misc.o: openmaxil/CMakeFiles/mmal_omxutil.dir/flags.make
openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_misc.o: interface/mmal/openmaxil/mmalomx_util_params_misc.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_misc.o"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_misc.o   -c /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params_misc.c

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_misc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_misc.i"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params_misc.c > CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_misc.i

openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_misc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_misc.s"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil/mmalomx_util_params_misc.c -o CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_misc.s

# Object files for target mmal_omxutil
mmal_omxutil_OBJECTS = \
"CMakeFiles/mmal_omxutil.dir/mmalomx_util_params.o" \
"CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_audio.o" \
"CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_video.o" \
"CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_camera.o" \
"CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_misc.o"

# External object files for target mmal_omxutil
mmal_omxutil_EXTERNAL_OBJECTS =

openmaxil/libmmal_omxutil.so: openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params.o
openmaxil/libmmal_omxutil.so: openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_audio.o
openmaxil/libmmal_omxutil.so: openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_video.o
openmaxil/libmmal_omxutil.so: openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_camera.o
openmaxil/libmmal_omxutil.so: openmaxil/CMakeFiles/mmal_omxutil.dir/mmalomx_util_params_misc.o
openmaxil/libmmal_omxutil.so: openmaxil/CMakeFiles/mmal_omxutil.dir/build.make
openmaxil/libmmal_omxutil.so: openmaxil/CMakeFiles/mmal_omxutil.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C shared library libmmal_omxutil.so"
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mmal_omxutil.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
openmaxil/CMakeFiles/mmal_omxutil.dir/build: openmaxil/libmmal_omxutil.so

.PHONY : openmaxil/CMakeFiles/mmal_omxutil.dir/build

openmaxil/CMakeFiles/mmal_omxutil.dir/clean:
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil && $(CMAKE_COMMAND) -P CMakeFiles/mmal_omxutil.dir/cmake_clean.cmake
.PHONY : openmaxil/CMakeFiles/mmal_omxutil.dir/clean

openmaxil/CMakeFiles/mmal_omxutil.dir/depend:
	cd /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/openmaxil /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil/CMakeFiles/mmal_omxutil.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : openmaxil/CMakeFiles/mmal_omxutil.dir/depend

